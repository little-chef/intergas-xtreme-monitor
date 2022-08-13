#include "esphome.h"
#include "esphome/core/defines.h"

#define TAG "Intergas Xtreme"

class IntergasHeaterMonitor : public PollingComponent {
    public:
        IntergasHeaterMonitor() : PollingComponent(500) {}

        // Settings for the ESP8266-nodemcuv2 board
#       ifdef USE_ARDUINO
#       ifdef USE_ESP8266
        void init_serial() {
            Serial.swap();
        }
#       endif
#       endif

        // Settings for the ESP32 board
#       ifdef USE_ESP32_FRAMEWORK_ARDUINO
#       define Serial Serial2
        void init_serial() {}
#       endif

        void stop_polling() {
            TextSensor_publish(monitor_status, "Stopped");
            ESP_LOGI(TAG, "Monitor stopped");
            init_serial();
            next_state = STOPPED;
        }

    private:
        enum ControlState {
            PRE_INIT,
            INIT,
            WAIT_CONNECTED,
            SEND_NEXT_COMMAND,
            PARSE_RESPONSE,
            STOPPED,
            FAILURE
        };
        ControlState next_state = PRE_INIT;

        typedef void (IntergasHeaterMonitor::*cmd_fptr)(std::string,
                                                        const std::vector<uint8_t> &);

        struct ids_command {
            std::string cmd;        // The command to send to the central heater.
            bool crc_needed;        // If the command requires a CRC byte to the command and the response
            int response_len;       // Expected number of data bytes in the response, without CRC field.
            cmd_fptr data_handler;  // Function pointer to the response handler of the command
            int period_secs;        // The period time in which to run this command
            time_t last_run;        // Always 0, updated at runtime.
        };

        std::vector<ids_command> ids_commands = {
            // Retrieve data from heater.
            { "REV",  false, 32, &IntergasHeaterMonitor::process_rev,          3600, 0 },
            { "REW",  true,  32, &IntergasHeaterMonitor::process_rew,          3600, 0 },
            { "B?\r", true,  32, &IntergasHeaterMonitor::process_prod_code,    3600, 0 },
            { "H0\r", true,  32, &IntergasHeaterMonitor::process_runtime_stats,  60, 0 },

            // Runtime metrics
            { "S?\r", true,  32, &IntergasHeaterMonitor::process_status,          5, 0 },
            { "S2\r", true,  32, &IntergasHeaterMonitor::process_status_extra,    5, 0 },
            { "D2\r", true,  32, &IntergasHeaterMonitor::process_status_extra_2,  5, 0 },
        };
        ids_command *current_command;

        ids_command *fetch_next_command() {
            static time_t last_run = 0;
            time_t now = millis();

            // Check for wraparound of the millis() clock, happens every 49 days
            if (now < last_run) {
                last_run = now;
                // reset the last run administration, and restart the schedule
                for (auto & ids_cmd : ids_commands) {
                    ids_cmd.last_run = 0;
                }
            }

            ids_command *next = nullptr;
            int32_t search_next_schedule = 0;

            for (auto & ids_cmd : ids_commands) {
                // If the last_run equals zero, just run it once. Typically only
                // run at boot like this, or after a wraparound of millis() clock
                if (ids_cmd.last_run == 0) {
                    next = &ids_cmd;
                    break;
                }

                time_t next_run_time =
                    ids_cmd.last_run + (ids_cmd.period_secs * 1000);

                if (now < next_run_time) {
                    // No time yet for this command to run
                    continue;
                }
                // The scheduler looks for the entry in the list that is beyond
                // the next scheduled time. The entry that is the most behind
                // that timestamp is the first one to run. This results in
                // a schedule where each command has at least the specified
                // period time between 2 runs.
                if ((now - next_run_time) > search_next_schedule) {
                    next = &ids_cmd;
                    search_next_schedule = (now - next_run_time);
                }
            }
            if (next) {
                next->last_run = now;
            }
            return next;
        }

        // Print a banner with library information.
        void banner() {
            ESP_LOGI(TAG, "ESPHome Intergas Xtreme IDS Monitor");
        }

        void setup() override {
            Serial.begin(9600);
            next_state = INIT;
        }

        // On every poll update, we do one single thing. Either send a command
        // to the central heater or parse the response. The goal is to keep the
        // update() cycle short, as in the background also other tasks need
        // to be executed, like for example the wifi network.
        // Serial port handler is implemented in hardware with interrupts. It
        // has a 255 byte sized buffer, which is much higher than the max of
        // 32 bytes we expect per command response.
        // By having the poll time in between the send and parse the response,
        // the central heater also has the poll-time interval to respond to commands.
        // That means the poll time cannot be too short. 2000 ms seems like
        // a good minimal the central heater can respond within.
        void update() override {
            switch (next_state) {
            case PRE_INIT:          next_state = state_pre_init();          break;
            case INIT:              next_state = state_init();              break;
            case WAIT_CONNECTED:    next_state = state_wait_connected();    break;
            case SEND_NEXT_COMMAND: next_state = state_send_next_command(); break;
            case PARSE_RESPONSE:    next_state = state_parse_respond();     break;
            case STOPPED:           next_state = state_stopped();           break;
            case FAILURE:           next_state = state_failed();            break;
            }
        }

        ControlState state_pre_init() {
            // Should not happen...
            ESP_LOGE(TAG, "Current State: PRE_INIT");
            return INIT;
        }

        ControlState state_init() {
            ESP_LOGI(TAG, "Current State: INIT");
            TextSensor_publish(monitor_status, "Init");
            banner();
            init_serial();
            return WAIT_CONNECTED;
        }

        ControlState state_wait_connected() {
            TextSensor_publish(monitor_status, "Wait connected");
            return SEND_NEXT_COMMAND;
        }

        bool verify_crc(std::vector<uint8_t> &byte_array) {
            uint8_t crc = 0;
            for (int i = 0; i < byte_array.size() - 1; i++) {
                crc ^= byte_array[i];
            }
            return crc == byte_array.back();
        }

        std::string add_command_crc(const std::string &cmd) {
            uint8_t crc = 0;
            for (std::string::size_type i = 0; i < cmd.size(); i++) {
                crc ^= cmd[i];
            }
            return cmd + std::string(1, crc);
        }

        ControlState state_send_next_command() {
            ids_command *ids_cmd = fetch_next_command();
            if (!ids_cmd) {
                // No command to schedule.
                return SEND_NEXT_COMMAND;
            }
            current_command = ids_cmd;
            switch_onboard_led(true);

            std::string log_cmd = get_log_cmd(ids_cmd->cmd);
            TextSensor_publish(monitor_status, (std::string("Fetching: " + log_cmd)).c_str());
            ESP_LOGD(TAG, "Send command %s", log_cmd.c_str());
            if (ids_cmd->crc_needed) {
                Serial.write(add_command_crc(ids_cmd->cmd).c_str());
            } else {
                Serial.write(ids_cmd->cmd.c_str());
            }
            switch_onboard_led(false);
            return PARSE_RESPONSE;
        }

        ControlState state_parse_respond() {
            switch_onboard_led(true);
            TextSensor_publish(monitor_status, "Processing...");
            ControlState next_state = SEND_NEXT_COMMAND;
            ids_command *ids_cmd = current_command;

            size_t len = Serial.available();

            // Check if enough data is received
            if (len < ids_cmd->response_len) {
                if (!len) {
                    ESP_LOGE(TAG, "No response received in time");
                } else {
                    ESP_LOGE(TAG, "Invalid status response received. Not enough data");
                }
                switch_onboard_led(false);
                return next_state;
            }

            std::vector<uint8_t> sbuf(len);
            Serial.readBytes(sbuf.data(), len);

            // Verify response checksum, that is the last byte of the data set
            if ((ids_cmd->crc_needed) && (!verify_crc(sbuf))) {
                ESP_LOGE(TAG, "Invalid %s response received. CRC error",
                    get_log_cmd(ids_cmd->cmd).c_str());
                ESP_LOGD(TAG, "Response %s Data: %s",
                    get_log_cmd(ids_cmd->cmd).c_str(),
                    format_hex_pretty(sbuf.data(), len).c_str());
                switch_onboard_led(false);
                return next_state;
            }

            // Call the function pointer
            cmd_fptr f = ids_cmd->data_handler;
            (this->*f)(ids_cmd->cmd, sbuf);
            switch_onboard_led(false);
            return next_state;
        }

        ControlState state_stopped() {
            TextSensor_publish(monitor_status, "Stopped");
            return STOPPED;
        }

        ControlState state_failed() {
            TextSensor_publish(monitor_status, "Failed");
            /* End state, no recovery until reset */
            return FAILURE;
        }

        std::string get_log_cmd(std::string cmd) {
            std::string repl = "\r";
            size_t i = cmd.find(repl);
            if (i == std::string::npos) {
                return cmd;
            }
            return cmd.replace(i, repl.length(), "\\r");
        }

        void log_response(std::string cmd, const std::vector<uint8_t> &sbuf) {
            ESP_LOGI(TAG, "Response %s Data: %s",
                get_log_cmd(cmd).c_str(),
                format_hex_pretty(sbuf.data(), sbuf.size()).c_str());
        }

        void process_status(std::string cmd, const std::vector<uint8_t> &sbuf) {
            // Retrieve the variables from the message
            double ch_pressure;
            bool ch_has_pressure_sensor;

            // Temperature sensors inside the Intergas Xtreme.
            Sensor_publish(temperature_heat_exchanger, getTemp(sbuf[1], sbuf[0]));
            Sensor_publish(temperature_flow, getTemp(sbuf[3], sbuf[2]));
            // Sensor_publish(temperature_return, getTemp(sbuf[5], sbuf[4])); Xtreme has no return temperature sensor.
            Sensor_publish(temperature_hot_water, getTemp(sbuf[7], sbuf[6]));
            Sensor_publish(temperature_cold_water, getTemp(sbuf[9],  sbuf[8]));
            Sensor_publish(temperature_flue_gas, getTemp(sbuf[11], sbuf[10]));
            Sensor_publish(temperature_setpoint, getTemp(sbuf[15], sbuf[14])); // Listed as T.max...
            Sensor_publish(temperature_outside, getTemp(sbuf[17],  sbuf[16]));
            Sensor_publish(temperature_boiler_to_heater, getTemp(sbuf[19],  sbuf[18]));

            ch_pressure = getFloat(sbuf[13], sbuf[12]);
            ch_has_pressure_sensor = get_bit(sbuf[28], 5);
            if (ch_has_pressure_sensor) {
                Sensor_publish(pressure, ch_pressure);
            }

            Sensor_publish(heater_io_current, getFloat(sbuf[23],  sbuf[22])); // +1

            if (get_bit(sbuf[27], 7)) {
                // current listed fault code is the active fault code
                TextSensor_publish(heater_fault_code, prettify_fault_code(sbuf[29]));
            } else {
                // There is no error active, indicate that.
                TextSensor_publish(heater_fault_code, prettify_fault_code(0xff));
            }
            // List it at least as an old fault code.
            TextSensor_publish(heater_last_fault_code, prettify_fault_code(sbuf[29]));

            std::string heater_status;
            switch (sbuf[24]) {
            case 51:  heater_status = "Hot water delivery"; break;
            case 0: case 102: heater_status = "Central Heating active"; break;
            case 126: heater_status = "Central Heating idle"; break;
            case 204: heater_status = "Hot water ramp down"; break;
            case 231: heater_status = "Central Heating ramp down"; break;
            default:  heater_status = "Code: " + esphome::to_string(sbuf[24]); break;
            }
            TextSensor_publish(heater_status_code, heater_status);

            // Process single bit values
            BinarySensor_publish(heater_gp_switch, get_bit(sbuf[26], 0));  // Ground pin ?
            BinarySensor_publish(heater_tap_switch, get_bit(sbuf[26], 1));
            BinarySensor_publish(heater_roomtherm, get_bit(sbuf[26], 2));
            BinarySensor_publish(heater_pump_running, get_bit(sbuf[26], 3));
            TextSensor_publish(heater_dwk, get_bit(sbuf[26], 4) ? "LT Zone" : "HT Zone");
            BinarySensor_publish(heater_alarm_status, get_bit(sbuf[26], 5));
            BinarySensor_publish(heater_cascade_relay, get_bit(sbuf[26], 6));
            BinarySensor_publish(heater_open_therm, get_bit(sbuf[26], 7));

            BinarySensor_publish(heater_gas_valve, get_bit(sbuf[28], 0));
            BinarySensor_publish(heater_spark, get_bit(sbuf[28], 1));
            BinarySensor_publish(heater_ionisation_signal, get_bit(sbuf[28], 2));
            BinarySensor_publish(heater_ot_disabled, get_bit(sbuf[28], 3));
            BinarySensor_publish(heater_has_low_water_pressure, get_bit(sbuf[28], 4));
            BinarySensor_publish(heater_burner_block, get_bit(sbuf[28], 6));
            BinarySensor_publish(heater_gradient_flag, get_bit(sbuf[28], 7));
        }

        void process_status_extra(std::string cmd, const std::vector<uint8_t> &sbuf) {
            Sensor_publish(tapflow, getFloat(sbuf[1], sbuf[0]));
            Sensor_publish(zone1_room_override, getFloat(sbuf[5], sbuf[4]));
            Sensor_publish(zone1_room_setpoint, getTemp(sbuf[7], sbuf[6]));
            Sensor_publish(zone1_room_temperature, getTemp(sbuf[9], sbuf[8]));
            Sensor_publish(zone2_room_override, getFloat(sbuf[11], sbuf[10]));
            Sensor_publish(zone2_room_setpoint, getTemp(sbuf[13], sbuf[12]));
            Sensor_publish(zone2_room_temperature, getTemp(sbuf[15], sbuf[14]));
            Sensor_publish(override_outside_temp, getTemp(sbuf[17], sbuf[16]));
        }

        void process_status_extra_2(std::string cmd, const std::vector<uint8_t> &sbuf) {
            Sensor_publish(fanspeed_set, getInt(sbuf[1],  sbuf[0]));
            Sensor_publish(fanspeed, getInt(sbuf[3],  sbuf[2]));
            Sensor_publish(pumpspeed_set, getInt(sbuf[11],  sbuf[10]));
            Sensor_publish(pumpspeed, getInt(sbuf[13],  sbuf[12]));
            // These values are valid, but not displayed in ESP Home
            //Sensor_publish(temperature_heat_exchanger_2, getTemp(sbuf[27], sbuf[26]));
            //Sensor_publish(valve_pos_set, getInt(sbuf[29],  sbuf[28]));
            //Sensor_publish(valve_pos, getInt(sbuf[31],  sbuf[30]));
        }

        void process_rev(std::string cmd, const std::vector<uint8_t> &sbuf) {
            // REV delivers these fields
            // Hardware Release string @ bytes 0-17
            // Software release string @ bytes 18-22
            // Byte 23 contains a '-' and is not used
            // ROM test 1 checksum @ bytes 24-27
            // ROM test 2 checksum @ bytes 28-31

            // export the fields from the back to the front of the array
            TextSensor_publish(rom_test_2_checksum, format_hex_pretty(&sbuf[28], 4).c_str());
            TextSensor_publish(rom_test_1_checksum, format_hex_pretty(&sbuf[24], 4).c_str());

            // Copy the buffer to a local array such that we can parse the strings.
            char copy[sbuf.size()];
            memcpy(copy, sbuf.data(), sbuf.size());
            copy[23] = '\0'; // Cut off SW release
            TextSensor_publish(software_release, &copy[18]);
            copy[18] = '\0'; // Cut off HW release
            TextSensor_publish(hardware_release, copy);
        }

        void process_rew(std::string cmd, const std::vector<uint8_t> &sbuf) {
            // REW delivers these fields for the DSP
            // Hardware Release string @ bytes 0-17
            // Software release string @ bytes 18-22
            // Byte 23 contains a '-' and is not used
            // ROM test 1 checksum @ bytes 24-27
            // ROM test 2 checksum @ bytes 28-31

            // export the fields from the back to the front of the array
            TextSensor_publish(dsp_rom_test_2_checksum, format_hex_pretty(&sbuf[28], 4).c_str());
            TextSensor_publish(dsp_rom_test_1_checksum, format_hex_pretty(&sbuf[24], 4).c_str());

            // Copy the buffer to a local array such that we can parse the strings.
            char copy[sbuf.size()];
            memcpy(copy, sbuf.data(), sbuf.size());
            copy[23] = '\0'; // Cut off SW release
            TextSensor_publish(dsp_software_release, &copy[18]);
            copy[18] = '\0'; // Cut off HW release
            TextSensor_publish(dsp_hardware_release, copy);
        }

        void process_prod_code(std::string cmd, const std::vector<uint8_t> &sbuf) {
            // B?\R delivers 32 bytes of data where only the bytes 25-31 contain valid data
            // Each hex value is converted to decimal and appended to the string representing
            // the whole production code. it is not a regular 7 byte hex value.
            // data '0A 04 08 03 00 0C 09' becomes: 10 04 08 3 0 12 09
            // Note that all values have leading zeroes, except the 4th + 5th value.
            char buf[50];
            snprintf(buf, sizeof(buf),
                "%02d%02d%02d%d%d%02d%02d",
                sbuf[25], sbuf[26], sbuf[27], sbuf[28],
                sbuf[29], sbuf[30], sbuf[31]);
            TextSensor_publish(production_code, buf);
        }

        void process_runtime_stats(std::string cmd, const std::vector<uint8_t> &sbuf) {
            Sensor_publish(line_power_connected_hours, getInt24(sbuf[30], sbuf[1], sbuf[0]));
            Sensor_publish(line_power_connected_count, getInt(sbuf[3], sbuf[2]));
            Sensor_publish(ch_function_hours, getInt(sbuf[5], sbuf[4]));
            Sensor_publish(dhw_function_hours, getInt(sbuf[7], sbuf[6]));
            Sensor_publish(burner_start_count, getInt24(sbuf[31], sbuf[9], sbuf[8]));
            Sensor_publish(ignition_failed, getInt(sbuf[11], sbuf[10]));
            Sensor_publish(flame_lost, getInt(sbuf[13], sbuf[12]));
            Sensor_publish(reset_count, getInt(sbuf[15], sbuf[14]));
            Sensor_publish(gas_meter_ch, getFloat32(sbuf[19], sbuf[18], sbuf[17], sbuf[16]));
            Sensor_publish(gas_meter_dhw, getFloat32(sbuf[23], sbuf[22], sbuf[21], sbuf[20]));
            Sensor_publish(water_meter, getFloat24(sbuf[28], sbuf[25], sbuf[24]));
            Sensor_publish(burner_starts_dhw_count, getInt24(sbuf[29], sbuf[27], sbuf[26]));
        }

        bool get_bit(uint8_t data, int bit) {
            return data & (1 << bit);
        }

        float getFloat24(byte msbh, byte msbl, byte lsb) {
            // Calculate float value from four bytes
            uint32_t _msbh = msbh;
            uint32_t _msbl = msbl;
            int32_t dword = (int32_t)(_msbh << 16 | _msbl << 8 | lsb);
            return ((float)dword) / 10000.0;
        }

        float getFloat32(byte msbh, byte msbl, byte lsbh, byte lsbl) {
            // Calculate float value from four bytes
            uint32_t _msbh = msbh;
            uint32_t _msbl = msbl;
            uint32_t _lsbh = lsbh;
            int32_t dword = (int32_t)(_msbh << 24 | _msbl << 16 | _lsbh << 8 | lsbl);
            return ((float)dword) / 10000.0;
        }

        float getFloat(byte msb, byte lsb) {
            // Calculate float value from two bytes
            uint16_t _msb = msb;
            int16_t word = (int16_t)(_msb << 8 | lsb);
            return ((float)word) / 100.0;
        }

        int16_t getInt(byte msb, byte lsb) {
            // Calculate float value from two bytes
            uint16_t _msb = msb;
            int16_t word = (int16_t)(_msb << 8 | lsb);
            return word;
        }

        int32_t getInt24(byte msbh, byte msbl, byte lsb) {
            // Calculate float value from two bytes
            uint32_t _msbh = msbh;
            uint32_t _msbl = msbl;
            int32_t word = (int32_t)(_msbh << 16 | _msbl << 8 | lsb);
            return word;
        }

        int8_t getSigned(byte lsb) {
            // Calculate float value from two bytes
            return (int8_t)lsb;
        }

        float getTemp(byte msb, byte lsb){
            uint16_t _msb = msb;
            int16_t word = (int16_t)(_msb << 8 | lsb);
            if ((word <= -5100) || (word == SHRT_MAX)) {
                // Intergas gives -5100 for disconnected sensors
                return nan("1");
            }
            return ((float)word) / 100.0;
        }

        template <typename V> void BinarySensor_publish(BinarySensor *sensor, V value) {
            publish_state(sensor, value);
        }

        template <typename V> void TextSensor_publish(TextSensor *sensor, V value) {
            publish_state(sensor, value);
        }

        template <typename V> void Sensor_publish(Sensor *sensor, V value) {
            publish_state(sensor, value);
        }

        // Send the value to Home-assistant, but only do that if the value
        // really has changed. A cache is used to determine changes. This lowers
        // the network bandwidth required, but also enables a smaller database
        // on the home assistant side while keeping a fast response to changing
        // values.
        template <typename S, typename V> void publish_state(S *sensor, V value) {
            static std::map<S*, V> cache = {};
            bool do_send_value = true;

            auto iter = cache.find(sensor);
            if (iter != cache.end()) {
                // Cached value exists for this sensor, check it the value has changed.
                if (is_equal(iter->second, value)) {
                    do_send_value = false;
                }
            }
            if (do_send_value) {
                sensor->publish_state(value);
                cache[sensor] = value;
            }
        }
        bool is_equal(const float &value1, const float &value2) {
            // For floating point also check if both values are NAN
            return ((value1 == value2) || (std::isnan(value1) && std::isnan(value2)));
        }
        bool is_equal(const char* &value1, const char* &value2) {
            return !strcmp(value1, value2);
        }
        bool is_equal(const std::string &value1, const std::string &value2) {
            return !value1.compare(value2);
        }

        void switch_onboard_led(const bool value) {
        // Settings for the ESP32 board
#       ifdef USE_ESP32_FRAMEWORK_ARDUINO
                BinaryOutput *output = static_cast<BinaryOutput *>(onboard_led);
                if (value) {
                    output->turn_on();
                } else {
                    output->turn_off();
                }
#       endif
        }

        std::string prettify_fault_code(uint8_t code) {
            std::string fault_string;
            switch (code) {
            case 0:  fault_string = "F000 - Sensor defect"; break;
            case 1:  fault_string = "F001 - Temperature too high during central heating demand"; break;
            case 2:  fault_string = "F002 - Temperature too high during domestic hot water (DHW) demand"; break;
            case 3:  fault_string = "F003 - Flue gas temperature too high"; break;
            case 4:  fault_string = "F004 - No flame during startup"; break;
            case 5:  fault_string = "F005 - Flame disappears during operation"; break;
            case 6:  fault_string = "F006 - Flame simulation error"; break;
            case 7:  fault_string = "F007 - No or insufficient ionisation flow"; break;
            case 8:  fault_string = "F008 - Fan speed incorrect"; break;
            case 9:  fault_string = "F009 - Burner controller has internal fault"; break;
            case 10: fault_string = "F010 - Sensor fault"; break;
            case 11: fault_string = "F011 - Sensor fault"; break;
            case 12: fault_string = "F012 - Sensor 5 fault"; break;
            case 14: fault_string = "F014 - Mounting fault sensor"; break;
            case 15: fault_string = "F015 - Mounting fault sensor S1"; break;
            case 16: fault_string = "F016 - Mounting fault S3"; break;
            case 18: fault_string = "F018 - Flue and/or air supply duct is blocked"; break;
            case 19: fault_string = "F019 - BMM error"; break;
            case 27: fault_string = "F027 - Short circuit of outdoor"; break;
            case 28: fault_string = "F028 - Reset error"; break;
            case 29: fault_string = "F029 - Gas valve error"; break;
            case 30: fault_string = "F030 - Sensor S3 fault"; break;
            case 31: fault_string = "F031 - Sensor fault S1"; break;
            case 0xff: fault_string = "No Fault detected"; break;
            default: fault_string = "Unspecified fault: " + esphome::to_string(code); break;
            }
            return fault_string;
        }
};
