#pragma once
#include "esphome.h"

namespace esphome {
namespace intergas_xtreme_monitor_component {

class IntergasXtremeMonitor : public PollingComponent {
    public:
        IntergasXtremeMonitor(
            binary_sensor::BinarySensor *heater_alarm_status,
            binary_sensor::BinarySensor *heater_burner_block,
            binary_sensor::BinarySensor *heater_cascade_relay,
            binary_sensor::BinarySensor *heater_gas_valve,
            binary_sensor::BinarySensor *heater_gp_switch,
            binary_sensor::BinarySensor *heater_gradient_flag,
            binary_sensor::BinarySensor *heater_has_low_water_pressure,
            binary_sensor::BinarySensor *heater_ionisation_signal,
            binary_sensor::BinarySensor *heater_open_therm,
            binary_sensor::BinarySensor *heater_ot_disabled,
            binary_sensor::BinarySensor *heater_pump_running,
            binary_sensor::BinarySensor *heater_roomtherm,
            binary_sensor::BinarySensor *heater_spark,
            binary_sensor::BinarySensor *heater_tap_switch,

            number::Number *anti_pendulum_time_heating,
            number::Number *comfort_keep_temperature,
            number::Number *comfort_offset,
            number::Number *control_temperature_boiler_operation,
            number::Number *delivery_temperature_control_panel,
            number::Number *dhw_temp_setting,
            number::Number *heatcurve_maximal_outside_temperature,
            number::Number *heatcurve_minimal_delivery_temperature,
            number::Number *heatcurve_minimal_outside_temperature,
            number::Number *load_adjustment,
            number::Number *lt_zone_setpoint_max,
            number::Number *max_heating_power,
            number::Number *max_setting_p050,
            number::Number *max_water_heating_power,
            number::Number *min_heating_delivery_temp_ot_and_rf,
            number::Number *min_heating_power,
            number::Number *min_water_heating_power,
            number::Number *nof_eco_days,
            number::Number *pump_afterrun_boiler,
            number::Number *pump_afterrun_central_heating,
            number::Number *pump_maximum,
            number::Number *pump_minimum,
            number::Number *start_rpm_heating,
            number::Number *start_speed_hot_water,
            number::Number *wait_time_heating_after_hot_water_delivery,
            number::Number *wait_time_heating_response,

            select::Select *activate_clock_program_ch,
            select::Select *dhw_timer,
            select::Select *display_option,
            select::Select *factory_reset,
            select::Select *function_alarm_relay,
            select::Select *function_ext1_temp_input,
            select::Select *function_ext2_temp_input,
            select::Select *function_relay_re1,
            select::Select *function_relay_re2,
            select::Select *function_x10_output,
            select::Select *hot_water_comfort_setting,
            select::Select *installation_type,
            select::Select *legionella_protection,
            select::Select *operation_mode_3_way_valve,
            select::Select *pump_setting,
            select::Select *response_OT_and_RF_thermostat,
            select::Select *step_modulation,
            select::Select *summer_winter_setting,
            select::Select *summer_winter_setting_user,

            sensor::Sensor *burner_start_count,
            sensor::Sensor *burner_starts_dhw_count,
            sensor::Sensor *ch_function_hours,
            sensor::Sensor *dhw_function_hours,
            sensor::Sensor *fanspeed,
            sensor::Sensor *fanspeed_set,
            sensor::Sensor *flame_lost,
            sensor::Sensor *gas_meter_ch,
            sensor::Sensor *gas_meter_dhw,
            sensor::Sensor *heater_io_current,
            sensor::Sensor *ignition_failed,
            sensor::Sensor *line_power_connected_count,
            sensor::Sensor *line_power_connected_hours,
            sensor::Sensor *override_outside_temp,
            sensor::Sensor *pressure,
            sensor::Sensor *pumpspeed,
            sensor::Sensor *pumpspeed_set,
            sensor::Sensor *reset_count,
            sensor::Sensor *tapflow,
            sensor::Sensor *temperature_boiler_to_heater,
            sensor::Sensor *temperature_cold_water,
            sensor::Sensor *temperature_flow,
            sensor::Sensor *temperature_flue_gas,
            sensor::Sensor *temperature_heat_exchanger,
            sensor::Sensor *temperature_hot_water,
            sensor::Sensor *temperature_outside,
            sensor::Sensor *temperature_setpoint,
            sensor::Sensor *water_meter,
            sensor::Sensor *zone1_room_override,
            sensor::Sensor *zone1_room_setpoint,
            sensor::Sensor *zone1_room_temperature,
            sensor::Sensor *zone2_room_override,
            sensor::Sensor *zone2_room_setpoint,
            sensor::Sensor *zone2_room_temperature,

            text_sensor::TextSensor *dsp_hardware_release,
            text_sensor::TextSensor *dsp_rom_test_1_checksum,
            text_sensor::TextSensor *dsp_rom_test_2_checksum,
            text_sensor::TextSensor *dsp_software_release,
            text_sensor::TextSensor *hardware_release,
            text_sensor::TextSensor *heater_dwk,
            text_sensor::TextSensor *heater_fault_code,
            text_sensor::TextSensor *heater_last_fault_code,
            text_sensor::TextSensor *heater_status_code,
            text_sensor::TextSensor *monitor_status,
            text_sensor::TextSensor *production_code,
            text_sensor::TextSensor *rom_test_1_checksum,
            text_sensor::TextSensor *rom_test_2_checksum,
            text_sensor::TextSensor *software_release,

            uart::UARTComponent *uart_boiler,

            output::BinaryOutput * onboard_led
        );
    
        void stop_polling();
        void set_param(uint8_t param, float value);
        void set_select_param(uint8_t param, select::Select *select, const char* text_value);
    
    protected:
        void setup() override;
        void update() override;
    
    private:
        const std::vector<int> list_xtreme_parameters{
                1,  2,  9, 10, 11, 12, 30, 31, 32, 33, 34, 35, 36,
                37, 38, 39, 40, 50, 51, 52, 53, 56, 57, 59, 60,
                70, 71, 72, 73, 74, 75, 76, 77, 78, 81, 85, 86, 87,
                90, 91, 97, 100, 101, 104
        };

        enum ControlState {
            PRE_INIT,
            INIT,
            WAIT_CONNECTED,
            SEND_NEXT_COMMAND,
            PARSE_READ_RESPONSE,
            PARSE_WRITE_RESPONSE,
            STOPPED,
            FAILURE
        };
        ControlState next_state = PRE_INIT;

        typedef void (IntergasXtremeMonitor::*cmd_fptr)(std::string,
                                                        const std::vector<uint8_t> &);

        // Data structure for data store-commands to send to the boiler. Typically
        // used for setting a specific parameter from ESP-home UI, like water
        // or heater temperatures. Those commands are queued in this queue to
        // be picked up later by the state machine to make sure to process them
        // in the proper sequence.
        struct store_command {
            std::string refresh_cmd;            // The command to run to refresh the values on the UI.
            std::vector<uint8_t> cmd_sequence;  // Command sequence to run
        };
        std::vector<store_command> store_commands = {};

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
            { "REV",  false, 32, &IntergasXtremeMonitor::process_rev,          3600, 0 },
            { "REW",  true,  32, &IntergasXtremeMonitor::process_rew,          3600, 0 },
            { "B?\r", true,  32, &IntergasXtremeMonitor::process_prod_code,    3600, 0 },
            { "H0\r", true,  32, &IntergasXtremeMonitor::process_runtime_stats,  60, 0 },

            // Runtime metrics
            { "S?\r", true,  32, &IntergasXtremeMonitor::process_status,          5, 0 },
            { "S2\r", true,  32, &IntergasXtremeMonitor::process_status_extra,    5, 0 },
            { "D2\r", true,  32, &IntergasXtremeMonitor::process_status_extra_2,  5, 0 },

            // Other values, just for logging
            { "H1\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "H2\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "H3\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "H4\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "H5\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "H6\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "H7\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "E0\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "G1\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "B1\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V0\r", true,  32, &IntergasXtremeMonitor::export_settings_v0,   3600, 0 },
            { "V1\r", true,  32, &IntergasXtremeMonitor::export_settings_v1,   3600, 0 },
            { "V2\r", true,  32, &IntergasXtremeMonitor::export_settings_v2,   3600, 0 },
            { "V3\r", true,  32, &IntergasXtremeMonitor::export_settings_v3,   3600, 0 },
            { "V4\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V5\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V6\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V7\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V8\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V9\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V:\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V;\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V<\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V=\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V>\r", true,  32, &IntergasXtremeMonitor::log_response,         3600, 0 },
            { "V?\r", true,  32, &IntergasXtremeMonitor::process_params,       3600, 0 },
            { "CRC",  true,  32, &IntergasXtremeMonitor::process_crc_register, 3600, 0 },
        };
        ids_command *current_command = nullptr;

        ids_command *fetch_next_command();
        ids_command *get_command(std::string cmd);
        void force_refresh_cmd(std::string cmd);

        // Print a banner with library information.
        void banner();

        ControlState state_pre_init();
        ControlState state_init();
        ControlState state_wait_connected();
        ControlState state_send_next_command();
        ControlState send_read_command();
        ControlState state_parse_read_respond();
        ControlState state_stopped();
        ControlState state_failed();
        ControlState send_write_command();
        ControlState state_parse_write_response();

        bool verify_crc(std::vector<uint8_t> &byte_array);
        std::vector<uint8_t> append_command_crc(const std::vector<uint8_t> &byte_array);
        std::string add_command_crc(const std::string &cmd);

        void store_parameter(uint8_t param_number, uint8_t value, std::string refresh_cmd);
        std::string get_log_cmd(std::string cmd);

        void log_response(std::string cmd, const std::vector<uint8_t> &sbuf);
        void export_settings(int register_base, std::string cmd, const std::vector<uint8_t> &sbuf);
        void export_settings_v0(std::string cmd, const std::vector<uint8_t> &sbuf);
        void export_settings_v1(std::string cmd, const std::vector<uint8_t> &sbuf);
        void export_settings_v2(std::string cmd, const std::vector<uint8_t> &sbuf);
        void export_settings_v3(std::string cmd, const std::vector<uint8_t> &sbuf);

        void process_status(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_status_extra(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_status_extra_2(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_crc_register(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_rev(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_rew(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_prod_code(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_runtime_stats(std::string cmd, const std::vector<uint8_t> &sbuf);
        void process_params(std::string cmd, const std::vector<uint8_t> &sbuf);

        bool get_bit(uint8_t data, int bit);
        float getFloat24(byte msbh, byte msbl, byte lsb);
        float getFloat32(byte msbh, byte msbl, byte lsbh, byte lsbl);
        float getFloat(byte msb, byte lsb);
        int16_t getInt(byte msb, byte lsb);
        int32_t getInt24(byte msbh, byte msbl, byte lsb);
        int8_t getSigned(byte lsb);
        float getTemp(byte msb, byte lsb);

        bool is_equal(const float &value1, const float &value2);
        bool is_equal(const char* &value1, const char* &value2);
        bool is_equal(const std::string &value1, const std::string &value2);

        void switch_onboard_led(const bool value);
        std::string prettify_fault_code(uint8_t code);

        template <typename V> void BinarySensor_publish(binary_sensor::BinarySensor *sensor, V value) {
            publish_state(sensor, value);
        }
        template <typename V> void TextSensor_publish(text_sensor::TextSensor *sensor, V value) {
            publish_state(sensor, value);
        }
        template <typename V> void Sensor_publish(sensor::Sensor *sensor, V value) {
            publish_state(sensor, value);
        }
        template <typename V> void Number_publish(number::Number *sensor, V value) {
            publish_state(sensor, value);
        }
        template <typename V> void Select_publish(select::Select *sensor, V value) {
            auto option = sensor->at(value);
            if (option.has_value()) {
                publish_state(sensor, option.value());
            } else {
                ESP_LOGE("main", "Value %d does not exist, dumping options:", value);
                for (size_t i = 0; i < sensor->size(); i++) {
                    auto option = sensor->at(i);
                    if (option.has_value()) {
                        ESP_LOGE("main", "Entry %d contains '%s'", i, option.value().c_str());
                    }
                }
            }
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

        binary_sensor::BinarySensor *heater_alarm_status;
        binary_sensor::BinarySensor *heater_burner_block;
        binary_sensor::BinarySensor *heater_cascade_relay;
        binary_sensor::BinarySensor *heater_gas_valve;
        binary_sensor::BinarySensor *heater_gp_switch;
        binary_sensor::BinarySensor *heater_gradient_flag;
        binary_sensor::BinarySensor *heater_has_low_water_pressure;
        binary_sensor::BinarySensor *heater_ionisation_signal;
        binary_sensor::BinarySensor *heater_open_therm;
        binary_sensor::BinarySensor *heater_ot_disabled;
        binary_sensor::BinarySensor *heater_pump_running;
        binary_sensor::BinarySensor *heater_roomtherm;
        binary_sensor::BinarySensor *heater_spark;
        binary_sensor::BinarySensor *heater_tap_switch;

        number::Number *anti_pendulum_time_heating;
        number::Number *comfort_keep_temperature;
        number::Number *comfort_offset;
        number::Number *control_temperature_boiler_operation;
        number::Number *delivery_temperature_control_panel;
        number::Number *dhw_temp_setting;
        number::Number *heatcurve_maximal_outside_temperature;
        number::Number *heatcurve_minimal_delivery_temperature;
        number::Number *heatcurve_minimal_outside_temperature;
        number::Number *load_adjustment;
        number::Number *lt_zone_setpoint_max;
        number::Number *max_heating_power;
        number::Number *max_setting_p050;
        number::Number *max_water_heating_power;
        number::Number *min_heating_delivery_temp_ot_and_rf;
        number::Number *min_heating_power;
        number::Number *min_water_heating_power;
        number::Number *nof_eco_days;
        number::Number *pump_afterrun_boiler;
        number::Number *pump_afterrun_central_heating;
        number::Number *pump_maximum;
        number::Number *pump_minimum;
        number::Number *start_rpm_heating;
        number::Number *start_speed_hot_water;
        number::Number *wait_time_heating_after_hot_water_delivery;
        number::Number *wait_time_heating_response;

        select::Select *activate_clock_program_ch;
        select::Select *dhw_timer;
        select::Select *display_option;
        select::Select *factory_reset;
        select::Select *function_alarm_relay;
        select::Select *function_ext1_temp_input;
        select::Select *function_ext2_temp_input;
        select::Select *function_relay_re1;
        select::Select *function_relay_re2;
        select::Select *function_x10_output;
        select::Select *hot_water_comfort_setting;
        select::Select *installation_type;
        select::Select *legionella_protection;
        select::Select *operation_mode_3_way_valve;
        select::Select *pump_setting;
        select::Select *response_OT_and_RF_thermostat;
        select::Select *step_modulation;
        select::Select *summer_winter_setting;
        select::Select *summer_winter_setting_user;

        sensor::Sensor *burner_start_count;
        sensor::Sensor *burner_starts_dhw_count;
        sensor::Sensor *ch_function_hours;
        sensor::Sensor *dhw_function_hours;
        sensor::Sensor *fanspeed;
        sensor::Sensor *fanspeed_set;
        sensor::Sensor *flame_lost;
        sensor::Sensor *gas_meter_ch;
        sensor::Sensor *gas_meter_dhw;
        sensor::Sensor *heater_io_current;
        sensor::Sensor *ignition_failed;
        sensor::Sensor *line_power_connected_count;
        sensor::Sensor *line_power_connected_hours;
        sensor::Sensor *override_outside_temp;
        sensor::Sensor *pressure;
        sensor::Sensor *pumpspeed;
        sensor::Sensor *pumpspeed_set;
        sensor::Sensor *reset_count;
        sensor::Sensor *tapflow;
        sensor::Sensor *temperature_boiler_to_heater;
        sensor::Sensor *temperature_cold_water;
        sensor::Sensor *temperature_flow;
        sensor::Sensor *temperature_flue_gas;
        sensor::Sensor *temperature_heat_exchanger;
        sensor::Sensor *temperature_hot_water;
        sensor::Sensor *temperature_outside;
        sensor::Sensor *temperature_setpoint;
        sensor::Sensor *water_meter;
        sensor::Sensor *zone1_room_override;
        sensor::Sensor *zone1_room_setpoint;
        sensor::Sensor *zone1_room_temperature;
        sensor::Sensor *zone2_room_override;
        sensor::Sensor *zone2_room_setpoint;
        sensor::Sensor *zone2_room_temperature;

        text_sensor::TextSensor *dsp_hardware_release;
        text_sensor::TextSensor *dsp_rom_test_1_checksum;
        text_sensor::TextSensor *dsp_rom_test_2_checksum;
        text_sensor::TextSensor *dsp_software_release;
        text_sensor::TextSensor *hardware_release;
        text_sensor::TextSensor *heater_dwk;
        text_sensor::TextSensor *heater_fault_code;
        text_sensor::TextSensor *heater_last_fault_code;
        text_sensor::TextSensor *heater_status_code;
        text_sensor::TextSensor *monitor_status;
        text_sensor::TextSensor *production_code;
        text_sensor::TextSensor *rom_test_1_checksum;
        text_sensor::TextSensor *rom_test_2_checksum;
        text_sensor::TextSensor *software_release;

        uart::UARTComponent *uart_boiler;

        output::BinaryOutput *onboard_led;

};

static IntergasXtremeMonitor *intergas_xtreme_monitor;

}  // namespace intergas_xtreme_monitor_component
}  // namespace esphome
