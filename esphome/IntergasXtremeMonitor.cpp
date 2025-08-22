#include "IntergasXtremeMonitor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace intergas_xtreme_monitor_component {

static constexpr const char* TAG = "Intergas Xtreme";

IntergasXtremeMonitor::IntergasXtremeMonitor(
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

    output::BinaryOutput *onboard_led

) : heater_alarm_status(heater_alarm_status),
    heater_burner_block(heater_burner_block),
    heater_cascade_relay(heater_cascade_relay),
    heater_gas_valve(heater_gas_valve),
    heater_gp_switch(heater_gp_switch),
    heater_gradient_flag(heater_gradient_flag),
    heater_has_low_water_pressure(heater_has_low_water_pressure),
    heater_ionisation_signal(heater_ionisation_signal),
    heater_open_therm(heater_open_therm),
    heater_ot_disabled(heater_ot_disabled),
    heater_pump_running(heater_pump_running),
    heater_roomtherm(heater_roomtherm),
    heater_spark(heater_spark),
    heater_tap_switch(heater_tap_switch),

    // Number *
    anti_pendulum_time_heating(anti_pendulum_time_heating),
    comfort_keep_temperature(comfort_keep_temperature),
    comfort_offset(comfort_offset),
    control_temperature_boiler_operation(control_temperature_boiler_operation),
    delivery_temperature_control_panel(delivery_temperature_control_panel),
    dhw_temp_setting(dhw_temp_setting),
    heatcurve_maximal_outside_temperature(heatcurve_maximal_outside_temperature),
    heatcurve_minimal_delivery_temperature(heatcurve_minimal_delivery_temperature),
    heatcurve_minimal_outside_temperature(heatcurve_minimal_outside_temperature),
    load_adjustment(load_adjustment),
    lt_zone_setpoint_max(lt_zone_setpoint_max),
    max_heating_power(max_heating_power),
    max_setting_p050(max_setting_p050),
    max_water_heating_power(max_water_heating_power),
    min_heating_delivery_temp_ot_and_rf(min_heating_delivery_temp_ot_and_rf),
    min_heating_power(min_heating_power),
    min_water_heating_power(min_water_heating_power),
    nof_eco_days(nof_eco_days),
    pump_afterrun_boiler(pump_afterrun_boiler),
    pump_afterrun_central_heating(pump_afterrun_central_heating),
    pump_maximum(pump_maximum),
    pump_minimum(pump_minimum),
    start_rpm_heating(start_rpm_heating),
    start_speed_hot_water(start_speed_hot_water),
    wait_time_heating_after_hot_water_delivery(wait_time_heating_after_hot_water_delivery),
    wait_time_heating_response(wait_time_heating_response),

    // Select *
    activate_clock_program_ch(activate_clock_program_ch),
    dhw_timer(dhw_timer),
    display_option(display_option),
    factory_reset(factory_reset),
    function_alarm_relay(function_alarm_relay),
    function_ext1_temp_input(function_ext1_temp_input),
    function_ext2_temp_input(function_ext2_temp_input),
    function_relay_re1(function_relay_re1),
    function_relay_re2(function_relay_re2),
    function_x10_output(function_x10_output),
    hot_water_comfort_setting(hot_water_comfort_setting),
    installation_type(installation_type),
    legionella_protection(legionella_protection),
    operation_mode_3_way_valve(operation_mode_3_way_valve),
    pump_setting(pump_setting),
    response_OT_and_RF_thermostat(response_OT_and_RF_thermostat),
    step_modulation(step_modulation),
    summer_winter_setting(summer_winter_setting),
    summer_winter_setting_user(summer_winter_setting_user),

    // Sensor *
    burner_start_count(burner_start_count),
    burner_starts_dhw_count(burner_starts_dhw_count),
    ch_function_hours(ch_function_hours),
    dhw_function_hours(dhw_function_hours),
    fanspeed(fanspeed),
    fanspeed_set(fanspeed_set),
    flame_lost(flame_lost),
    gas_meter_ch(gas_meter_ch),
    gas_meter_dhw(gas_meter_dhw),
    heater_io_current(heater_io_current),
    ignition_failed(ignition_failed),
    line_power_connected_count(line_power_connected_count),
    line_power_connected_hours(line_power_connected_hours),
    override_outside_temp(override_outside_temp),
    pressure(pressure),
    pumpspeed(pumpspeed),
    pumpspeed_set(pumpspeed_set),
    reset_count(reset_count),
    tapflow(tapflow),
    temperature_boiler_to_heater(temperature_boiler_to_heater),
    temperature_cold_water(temperature_cold_water),
    temperature_flow(temperature_flow),
    temperature_flue_gas(temperature_flue_gas),
    temperature_heat_exchanger(temperature_heat_exchanger),
    temperature_hot_water(temperature_hot_water),
    temperature_outside(temperature_outside),
    temperature_setpoint(temperature_setpoint),
    water_meter(water_meter),
    zone1_room_override(zone1_room_override),
    zone1_room_setpoint(zone1_room_setpoint),
    zone1_room_temperature(zone1_room_temperature),
    zone2_room_override(zone2_room_override),
    zone2_room_setpoint(zone2_room_setpoint),
    zone2_room_temperature(zone2_room_temperature),

    // TextSensor *
    dsp_hardware_release(dsp_hardware_release),
    dsp_rom_test_1_checksum(dsp_rom_test_1_checksum),
    dsp_rom_test_2_checksum(dsp_rom_test_2_checksum),
    dsp_software_release(dsp_software_release),
    hardware_release(hardware_release),
    heater_dwk(heater_dwk),
    heater_fault_code(heater_fault_code),
    heater_last_fault_code(heater_last_fault_code),
    heater_status_code(heater_status_code),
    monitor_status(monitor_status),
    production_code(production_code),
    rom_test_1_checksum(rom_test_1_checksum),
    rom_test_2_checksum(rom_test_2_checksum),
    software_release(software_release),
    // UARTDevice *
    uart_boiler(uart_boiler),
    onboard_led(onboard_led)
{
    next_state = INIT;
}

void IntergasXtremeMonitor::stop_polling() {
    TextSensor_publish(monitor_status, "Stopped");
    ESP_LOGI(TAG, "Monitor stopped");
    next_state = STOPPED;
}

void IntergasXtremeMonitor::set_param(uint8_t param, float value) {
    std::string reg = "V" + esphome::to_string(param/32) + "\r";
    store_parameter(param, static_cast<uint8_t>(value), reg);
}

auto IntergasXtremeMonitor::set_select_param(uint8_t param, select::Select *select, const char* text_value) -> void {
    auto index = select->index_of(text_value);
    if (index.has_value()) {
        set_param(param, index.value());
    } else {
        ESP_LOGE("main", "There is no option '%s' for param %d", text_value, param);
    }
}

auto IntergasXtremeMonitor::fetch_next_command() -> ids_command* {
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

IntergasXtremeMonitor::ids_command *IntergasXtremeMonitor::get_command(std::string cmd) {
    for (auto & ids_cmd : ids_commands) {
        if (ids_cmd.cmd == cmd) {
            return &ids_cmd;
        }
    }
    return nullptr;
}

void IntergasXtremeMonitor::force_refresh_cmd(std::string cmd) {
    ids_command * ids_cmd = get_command(cmd);
    if (!ids_cmd) {
        ESP_LOGE(TAG, "Command %s does not exist", cmd.c_str());
    } else {
        ids_cmd->last_run = 0; // Force the last run to be outdated.
    }
}

// Print a banner with library information.
void IntergasXtremeMonitor::banner() {
    ESP_LOGI(TAG, "ESPHome Intergas Xtreme IDS Monitor");
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
void IntergasXtremeMonitor::update() {
    switch (next_state) {
    case INIT:                  next_state = state_init();                  break;
    case WAIT_CONNECTED:        next_state = state_wait_connected();        break;
    case SEND_NEXT_COMMAND:     next_state = state_send_next_command();     break;
    case PARSE_READ_RESPONSE:   next_state = state_parse_read_respond();    break;
    case PARSE_WRITE_RESPONSE:  next_state = state_parse_write_response();  break;
    case STOPPED:               next_state = state_stopped();               break;
    case FAILURE:               next_state = state_failed();                break;
    }
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_init() {
    ESP_LOGI(TAG, "Current State: INIT");
    TextSensor_publish(monitor_status, "Init");
    banner();
    return WAIT_CONNECTED;
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_wait_connected() {
    TextSensor_publish(monitor_status, "Wait connected");
    return SEND_NEXT_COMMAND;
}

bool IntergasXtremeMonitor::verify_crc(std::vector<uint8_t> &byte_array) {
    uint8_t crc = 0;
    // Parse the whole byte_array, except last value as that contains
    // the expected CRC value.
    for (int i = 0; i < byte_array.size() - 1; i++) {
        crc ^= byte_array[i];
    }
    return crc == byte_array.back();
}

std::vector<uint8_t> IntergasXtremeMonitor::append_command_crc(const std::vector<uint8_t> &byte_array) {
    uint8_t crc = 0;
    std::vector<uint8_t> sbuf = byte_array;

    for (uint8_t a_byte : sbuf) {
        crc ^= a_byte;
    }
    sbuf.push_back(crc);
    return sbuf;
}

std::string IntergasXtremeMonitor::add_command_crc(const std::string &cmd) {
    uint8_t crc = 0;
    for (std::string::size_type i = 0; i < cmd.size(); i++) {
        crc ^= cmd[i];
    }
    return cmd + std::string(1, crc);
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_send_next_command() {
    // Storing new settings always has highest priorities above
    // regular updates.
    if (store_commands.size()) {
        return send_write_command();
    }
    return send_read_command();
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::send_read_command() {
    ids_command *ids_cmd = fetch_next_command();
    if (!ids_cmd) {
        // No command to schedule.
        ESP_LOGI(TAG, "Monitor Idle");
        return SEND_NEXT_COMMAND;
    }
    TextSensor_publish(monitor_status, "Operational");

    current_command = ids_cmd;
    switch_onboard_led(true);

    std::string log_cmd = get_log_cmd(ids_cmd->cmd);
    ESP_LOGD(TAG, "Send command %s", log_cmd.c_str());
    if (ids_cmd->crc_needed) {
        id(uart_boiler).write_str(add_command_crc(ids_cmd->cmd).c_str());
    } else {
        id(uart_boiler).write_str(ids_cmd->cmd.c_str());
    }
    return PARSE_READ_RESPONSE;
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_parse_read_respond() {
    ControlState set_next_state = SEND_NEXT_COMMAND;
    ids_command *ids_cmd = current_command;

    size_t len = id(uart_boiler).available();

    // Check if enough data is received
    if (len < ids_cmd->response_len) {
        if (!len) {
            ESP_LOGE(TAG, "No response received in time");
        } else {
            ESP_LOGE(TAG, "Invalid status response received. Not enough data");
        }
        switch_onboard_led(false);
        return set_next_state;
    }

    std::vector<uint8_t> sbuf(len);
    id(uart_boiler).read_array(sbuf.data(), len);

    // Verify response checksum, that is the last byte of the data set
    if ((ids_cmd->crc_needed) && (!verify_crc(sbuf))) {
        ESP_LOGE(TAG, "Invalid %s response received. CRC error",
            get_log_cmd(ids_cmd->cmd).c_str());
        ESP_LOGD(TAG, "Response %s Data: %s",
            get_log_cmd(ids_cmd->cmd).c_str(),
            format_hex_pretty(sbuf.data(), len).c_str());
        switch_onboard_led(false);
        return set_next_state;
    }

    // Call the function pointer
    cmd_fptr f = ids_cmd->data_handler;
    (this->*f)(ids_cmd->cmd, sbuf);
    switch_onboard_led(false);
    return set_next_state;
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_stopped() {
    TextSensor_publish(monitor_status, "Stopped");
    return STOPPED;
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_failed() {
    TextSensor_publish(monitor_status, "Failed");
    /* End state, no recovery until reset */
    return FAILURE;
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::send_write_command() {
    TextSensor_publish(monitor_status, "Write setting");
    switch_onboard_led(true);

    store_command pcmd = *(store_commands.begin());

    ESP_LOGI(TAG, "Write parameter via command: %s, len: %d",
        format_hex_pretty(pcmd.cmd_sequence.data(), pcmd.cmd_sequence.size()).c_str(),
        pcmd.cmd_sequence.size());

    id(uart_boiler).write_array(pcmd.cmd_sequence.data(), pcmd.cmd_sequence.size());

    force_refresh_cmd(pcmd.refresh_cmd);

    return PARSE_WRITE_RESPONSE;
}

IntergasXtremeMonitor::ControlState IntergasXtremeMonitor::state_parse_write_response() {
    TextSensor_publish(monitor_status, "Processing write");

    store_command pcmd = *(store_commands.begin());

    ESP_LOGI(TAG, "Parsing response for store command: %s, len: %d",
        format_hex_pretty(pcmd.cmd_sequence.data(), pcmd.cmd_sequence.size()).c_str(),
        pcmd.cmd_sequence.size());

    size_t len = id(uart_boiler).available();
    if (len) {
        std::vector<uint8_t> sbuf(len);
        id(uart_boiler).read_array(sbuf.data(), len);

        ESP_LOGI(TAG, "Store response: %s",
            format_hex_pretty(sbuf.data(), sbuf.size()).c_str());
    } else{
            ESP_LOGE(TAG, "No response received");
    }

    // Erase the command from the queue that e just finished processing
    store_commands.erase(store_commands.begin());

    switch_onboard_led(false);
    return SEND_NEXT_COMMAND;
}

void IntergasXtremeMonitor::store_parameter(uint8_t param_number, uint8_t value, std::string refresh_cmd) {
    std::vector<uint8_t> sbuf{ 'P', param_number, value };
    sbuf = append_command_crc(sbuf);
    store_command pcmd;
    pcmd.refresh_cmd = refresh_cmd;
    pcmd.cmd_sequence = sbuf;
    store_commands.push_back(pcmd);
    ESP_LOGI(TAG, "Storing parameter P%03d value: %d", param_number, value);
}

std::string IntergasXtremeMonitor::get_log_cmd(std::string cmd) {
    std::string repl = "\r";
    size_t i = cmd.find(repl);
    if (i == std::string::npos) {
        return cmd;
    }
    return cmd.replace(i, repl.length(), "\\r");
}

void IntergasXtremeMonitor::log_response(std::string cmd, const std::vector<uint8_t> &sbuf) {
    ESP_LOGI(TAG, "Response %s Data: %s",
        get_log_cmd(cmd).c_str(),
        format_hex_pretty(sbuf.data(), sbuf.size()).c_str());
}

void IntergasXtremeMonitor::export_settings(int register_base, std::string cmd, const std::vector<uint8_t> &sbuf) {
    for (int i = 0; i < sbuf.size() - 1; i++) {
        int parameter_id = (register_base * 32) + i;

        switch(parameter_id) {
            case 1:
                Select_publish(installation_type, getSigned(sbuf[i]));
                break;
            case 2:
                Select_publish(display_option, getSigned(sbuf[i]));
                break;
            case 9:
                Number_publish(load_adjustment, getSigned(sbuf[i]));
                break;
            case 10:
                Number_publish(max_heating_power, getSigned(sbuf[i]));
                break;
            case 11:
                Number_publish(min_heating_power, getSigned(sbuf[i]));
                break;
            case 12:
                Number_publish(start_rpm_heating, getSigned(sbuf[i]));
                break;
            case 30:
                Select_publish(pump_setting, getSigned(sbuf[i]));
                break;
            case 31:
                Number_publish(pump_maximum, getSigned(sbuf[i]));
                break;
            case 32:
                Number_publish(pump_minimum, getSigned(sbuf[i]));
                break;
            case 33:
                Number_publish(pump_afterrun_central_heating, getSigned(sbuf[i]));
                break;
            case 34:
                Number_publish(pump_afterrun_boiler, getSigned(sbuf[i]));
                break;
            case 35:
                Select_publish(step_modulation, getSigned(sbuf[i]));
                break;
            case 36:
                Number_publish(anti_pendulum_time_heating, getSigned(sbuf[i]));
                break;
            case 37:
                Number_publish(wait_time_heating_response, getSigned(sbuf[i]));
                break;
            case 38:
                Select_publish(summer_winter_setting, getSigned(sbuf[i]));
                break;
            case 39:
                Select_publish(summer_winter_setting_user, getSigned(sbuf[i]));
                break;
            case 40:
                Select_publish(activate_clock_program_ch, getSigned(sbuf[i]));
                break;
            case 50:
                Number_publish(delivery_temperature_control_panel, getSigned(sbuf[i]));
                break;
            case 51:
                Number_publish(heatcurve_minimal_delivery_temperature, getSigned(sbuf[i]));
                break;
            case 52:
                Number_publish(heatcurve_minimal_outside_temperature, getSigned(sbuf[i]));
                break;
            case 53:
                Number_publish(heatcurve_maximal_outside_temperature, getSigned(sbuf[i]));
                break;
            case 56:
                Number_publish(min_heating_delivery_temp_ot_and_rf, getSigned(sbuf[i]));
                break;
            case 57:
                Select_publish(response_OT_and_RF_thermostat, getSigned(sbuf[i]));
                break;
            case 59:
                Number_publish(max_setting_p050, getSigned(sbuf[i]));
                break;
            case 60:
                Number_publish(lt_zone_setpoint_max, getSigned(sbuf[i]));
                break;
            case 70:
                Number_publish(max_water_heating_power, getSigned(sbuf[i]));
                break;
            case 71:
                Number_publish(min_water_heating_power, getSigned(sbuf[i]));
                break;
            case 72:
                Number_publish(start_speed_hot_water, getSigned(sbuf[i]));
                break;
            case 73:
                Number_publish(comfort_keep_temperature, getSigned(sbuf[i]));
                break;
            case 74:
                Number_publish(nof_eco_days, getSigned(sbuf[i]));
                break;
            case 75:
                Number_publish(control_temperature_boiler_operation, getSigned(sbuf[i]));
                break;
            case 76:
                Select_publish(hot_water_comfort_setting, getSigned(sbuf[i]));
                break;
            case 77:
                Number_publish(wait_time_heating_after_hot_water_delivery, getSigned(sbuf[i]));
                break;
            case 78:
                Number_publish(dhw_temp_setting, getSigned(sbuf[i]));
                break;
            case 81:
                Select_publish(operation_mode_3_way_valve, getSigned(sbuf[i]));
                break;
            case 85:
                Select_publish(legionella_protection, getSigned(sbuf[i]));
                break;
            case 86:
                Number_publish(comfort_offset, getSigned(sbuf[i]));
                break;
            case 87:
                Select_publish(dhw_timer, getSigned(sbuf[i]));
                break;
            case 90:
                Select_publish(function_relay_re1, getSigned(sbuf[i]));
                break;
            case 91:
                Select_publish(function_relay_re2, getSigned(sbuf[i]));
                break;
            case 97:
                Select_publish(function_alarm_relay, getSigned(sbuf[i]));
                break;
            case 100:
                Select_publish(function_ext1_temp_input, getSigned(sbuf[i]));
                break;
            case 101:
                Select_publish(function_ext2_temp_input, getSigned(sbuf[i]));
                break;
            case 104:
                Select_publish(function_x10_output, getSigned(sbuf[i]));
                break;
            case 255:
                Select_publish(factory_reset, getSigned(sbuf[i]));
                break;
            default:
                // Check if the parameter is a known value, if not ignore it.
                if (std::find(
                        list_xtreme_parameters.begin(),
                        list_xtreme_parameters.end(),
                        parameter_id) != list_xtreme_parameters.end()) {
                    ESP_LOGI(TAG, "Parameter P%03d has value: %d",
                        parameter_id, getSigned(sbuf[i]));
                }
        }
    }
}

void IntergasXtremeMonitor::export_settings_v0(std::string cmd, const std::vector<uint8_t> &sbuf) {
    export_settings(0, cmd, sbuf);
}
void IntergasXtremeMonitor::export_settings_v1(std::string cmd, const std::vector<uint8_t> &sbuf) {
    export_settings(1, cmd, sbuf);
}
void IntergasXtremeMonitor::export_settings_v2(std::string cmd, const std::vector<uint8_t> &sbuf) {
    export_settings(2, cmd, sbuf);
}
void IntergasXtremeMonitor::export_settings_v3(std::string cmd, const std::vector<uint8_t> &sbuf) {
    export_settings(3, cmd, sbuf);
}

void IntergasXtremeMonitor::process_status(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
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
    unsigned u_heater_status = sbuf[24];
    switch (u_heater_status) {
    case 51:  heater_status = "Hot water delivery"; break;
    case 0: case 102: heater_status = "Central Heating active"; break;
    case 126: heater_status = "Central Heating idle"; break;
    case 204: heater_status = "Hot water ramp down"; break;
    case 231: heater_status = "Central Heating ramp down"; break;
    default:  heater_status = "Code: " + esphome::to_string(u_heater_status); break;
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

void IntergasXtremeMonitor::process_status_extra(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
    Sensor_publish(tapflow, getFloat(sbuf[1], sbuf[0]));
    Sensor_publish(zone1_room_override, getFloat(sbuf[5], sbuf[4]));
    Sensor_publish(zone1_room_setpoint, getTemp(sbuf[7], sbuf[6]));
    Sensor_publish(zone1_room_temperature, getTemp(sbuf[9], sbuf[8]));
    Sensor_publish(zone2_room_override, getFloat(sbuf[11], sbuf[10]));
    Sensor_publish(zone2_room_setpoint, getTemp(sbuf[13], sbuf[12]));
    Sensor_publish(zone2_room_temperature, getTemp(sbuf[15], sbuf[14]));
    Sensor_publish(override_outside_temp, getTemp(sbuf[17], sbuf[16]));
}

void IntergasXtremeMonitor::process_status_extra_2(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
    Sensor_publish(fanspeed_set, getInt(sbuf[1],  sbuf[0]));
    Sensor_publish(fanspeed, getInt(sbuf[3],  sbuf[2]));
    Sensor_publish(pumpspeed_set, getInt(sbuf[11],  sbuf[10]));
    Sensor_publish(pumpspeed, getInt(sbuf[13],  sbuf[12]));
    // These values are valid, but not displayed in ESP Home
    //Sensor_publish(temperature_heat_exchanger_2, getTemp(sbuf[27], sbuf[26]));
    //Sensor_publish(valve_pos_set, getInt(sbuf[29],  sbuf[28]));
    //Sensor_publish(valve_pos, getInt(sbuf[31],  sbuf[30]));
}

void IntergasXtremeMonitor::process_crc_register(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
    /*
    Sensor_publish(interrupt_time, float(getInt(sbuf[1], sbuf[0])) / 5.0);
    Sensor_publish(interrupt_load, float(getInt(sbuf[3], sbuf[2])) / 6.25);
    Sensor_publish(main_load, float(getInt(sbuf[5], sbuf[4])) / 8.0);
    int16_t freq_number = getInt(sbuf[7], sbuf[6]);
    if (freq_number > 0) {
        Sensor_publish(net_frequency, 2000.0 / float(freq_number));
    } else {
        Sensor_publish(net_frequency, float(0));
    }
    Sensor_publish(voltage_reference, float(getInt(sbuf[9], sbuf[8])) * 5.0 / 1024.0);
    */
}

void IntergasXtremeMonitor::process_rev(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
    // REV delivers these fields
    // Hardware Release string @ bytes 0-17
    // Software release string @ bytes 18-22
    // Byte 23 contains a '-' and is not used
    // ROM test 1 checksum @ bytes 24-27
    // ROM test 2 checksum @ bytes 28-31

    // export the fields from the back to the front of the array
    TextSensor_publish(rom_test_2_checksum, format_hex_pretty(sbuf.data() + 28, 4).c_str());
    TextSensor_publish(rom_test_1_checksum, format_hex_pretty(sbuf.data() + 24, 4).c_str());

    // Copy the buffer to a local array such that we can parse the strings.
    char copy[sbuf.size()];
    memcpy(copy, sbuf.data(), sbuf.size());

    copy[23] = '\0'; // Cut off SW release
    TextSensor_publish(software_release, copy + 18);
    copy[18] = '\0'; // Cut off HW release
    TextSensor_publish(hardware_release, copy);
}

void IntergasXtremeMonitor::process_rew(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
    // REW delivers these fields for the DSP
    // Hardware Release string @ bytes 0-17
    // Software release string @ bytes 18-22
    // Byte 23 contains a '-' and is not used
    // ROM test 1 checksum @ bytes 24-27
    // ROM test 2 checksum @ bytes 28-31

    // Copy the buffer to a local array such that we can parse the strings.
    char copy[sbuf.size()];
    memcpy(copy, sbuf.data(), sbuf.size());

    // export the fields from the back to the front of the array
    TextSensor_publish(dsp_rom_test_2_checksum, format_hex_pretty(sbuf.data() + 28, 4).c_str());
    TextSensor_publish(dsp_rom_test_1_checksum, format_hex_pretty(sbuf.data() + 24, 4).c_str());

    copy[23] = '\0'; // Cut off SW release
    TextSensor_publish(dsp_software_release, copy + 18);
    copy[18] = '\0'; // Cut off HW release
    TextSensor_publish(dsp_hardware_release, copy);
}

void IntergasXtremeMonitor::process_prod_code(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
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

void IntergasXtremeMonitor::process_runtime_stats(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
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

void IntergasXtremeMonitor::process_params(std::string cmd, const std::vector<uint8_t> &sbuf) {
    log_response(cmd, sbuf);
    ESP_LOGI(TAG, "heater_on: %i", getSigned(sbuf[0]));
    ESP_LOGI(TAG, "comfort_mode: %i", getSigned(sbuf[1]));
    ESP_LOGI(TAG, "ch_set_max: %i", getSigned(sbuf[2]));
    ESP_LOGI(TAG, "dhw_set: %i", getSigned(sbuf[3]));
    ESP_LOGI(TAG, "eco_days: %i", getSigned(sbuf[4]));
    ESP_LOGI(TAG, "comfort_set: %i", getSigned(sbuf[5]));
    ESP_LOGI(TAG, "dhw_at_night: %i", getSigned(sbuf[6]));
    ESP_LOGI(TAG, "ch_at_night: %i", getSigned(sbuf[7]));
    ESP_LOGI(TAG, "param_1: %i", getSigned(sbuf[8]));
    ESP_LOGI(TAG, "param_2: %i", getSigned(sbuf[9]));
    ESP_LOGI(TAG, "param_3: %i", getSigned(sbuf[10]));
    ESP_LOGI(TAG, "param_4: %i", getSigned(sbuf[11]));
    ESP_LOGI(TAG, "param_5: %i", getSigned(sbuf[12]));
    ESP_LOGI(TAG, "param_6: %i", getSigned(sbuf[13]));
    ESP_LOGI(TAG, "param_7: %i", getSigned(sbuf[14]));
    ESP_LOGI(TAG, "param_8: %i", getSigned(sbuf[15]));
    ESP_LOGI(TAG, "param_9: %i", getSigned(sbuf[16]));
    ESP_LOGI(TAG, "param_A: %i", getSigned(sbuf[17]));
    ESP_LOGI(TAG, "param_b: %i", getSigned(sbuf[18]));
    ESP_LOGI(TAG, "param_C: %i", getSigned(sbuf[19]));
    ESP_LOGI(TAG, "param_c: %i", getSigned(sbuf[20]));
    ESP_LOGI(TAG, "param_d: %i", getSigned(sbuf[21]));
    ESP_LOGI(TAG, "param_E: %i", getSigned(sbuf[22]));
    ESP_LOGI(TAG, "param_E.: %i", getSigned(sbuf[23]));
    ESP_LOGI(TAG, "param_F: %i", getSigned(sbuf[24]));
    ESP_LOGI(TAG, "param_H: %i", getSigned(sbuf[25]));
    ESP_LOGI(TAG, "param_n: %i", getSigned(sbuf[26]));
    ESP_LOGI(TAG, "param_o: %i", getSigned(sbuf[27]));
    ESP_LOGI(TAG, "param_P: %i", getSigned(sbuf[28]));
    ESP_LOGI(TAG, "param_r: %i", getSigned(sbuf[29]));
    ESP_LOGI(TAG, "param_F.: %i", getSigned(sbuf[30]));
}

bool IntergasXtremeMonitor::get_bit(uint8_t data, int bit) {
    return data & (1 << bit);
}

float IntergasXtremeMonitor::getFloat24(const uint8_t msbh, const uint8_t msbl, const uint8_t lsb) {
    // Calculate float value from four bytes
    uint32_t _msbh = msbh;
    uint32_t _msbl = msbl;
    int32_t dword = (int32_t)(_msbh << 16 | _msbl << 8 | lsb);
    return ((float)dword) / 10000.0;
}

float IntergasXtremeMonitor::getFloat32(const uint8_t msbh, const uint8_t msbl, const uint8_t lsbh, const uint8_t lsbl) {
    // Calculate float value from four bytes
    uint32_t _msbh = msbh;
    uint32_t _msbl = msbl;
    uint32_t _lsbh = lsbh;
    int32_t dword = (int32_t)(_msbh << 24 | _msbl << 16 | _lsbh << 8 | lsbl);
    return ((float)dword) / 10000.0;
}

float IntergasXtremeMonitor::getFloat(const uint8_t msb, const uint8_t lsb) {
    // Calculate float value from two bytes
    uint16_t _msb = msb;
    int16_t word = (int16_t)(_msb << 8 | lsb);
    return ((float)word) / 100.0;
}

int16_t IntergasXtremeMonitor::getInt(const uint8_t msb, const uint8_t lsb) {
    // Calculate integer value from two bytes
    uint16_t _msb = msb;
    int16_t word = (int16_t)(_msb << 8 | lsb);
    return word;
}

int32_t IntergasXtremeMonitor::getInt24(const uint8_t msbh, const uint8_t msbl, const uint8_t lsb) {
    // Calculate 24 bit integer value from two bytes
    uint32_t _msbh = msbh;
    uint32_t _msbl = msbl;
    int32_t word = (int32_t)(_msbh << 16 | _msbl << 8 | lsb);
    return word;
}

int8_t IntergasXtremeMonitor::getSigned(const uint8_t lsb) {
    // Calculate signed value from one byte
    return (int8_t)lsb;
}

float IntergasXtremeMonitor::getTemp(const uint8_t msb, const uint8_t lsb){
    uint16_t _msb = msb;
    int16_t word = (int16_t)(_msb << 8 | lsb);
    if ((word <= -5100) || (word == SHRT_MAX)) {
        // Intergas gives -5100 for disconnected sensors
        return nan("1");
    }
    return ((float)word) / 100.0;
}

bool IntergasXtremeMonitor::is_equal(const float &value1, const float &value2) {
    // For floating point also check if both values are NAN
    return ((value1 == value2) || (std::isnan(value1) && std::isnan(value2)));
}
bool IntergasXtremeMonitor::is_equal(const char* &value1, const char* &value2) {
    return !strcmp(value1, value2);
}
bool IntergasXtremeMonitor::is_equal(const std::string &value1, const std::string &value2) {
    return !value1.compare(value2);
}

void IntergasXtremeMonitor::switch_onboard_led(const bool value) {
    // Settings for the ESP32 board
    output::BinaryOutput *output = static_cast<output::BinaryOutput *>(onboard_led);
    if (value) {
        output->turn_on();
    } else {
        output->turn_off();
    }
}

std::string IntergasXtremeMonitor::prettify_fault_code(uint8_t code) {
    unsigned u_code = code;
    switch (u_code) {
    case 0:  return "F000 - Sensor defect";
    case 1:  return "F001 - Temperature too high during central heating demand";
    case 2:  return "F002 - Temperature too high during domestic hot water (DHW) demand";
    case 3:  return "F003 - Flue gas temperature too high";
    case 4:  return "F004 - No flame during startup";
    case 5:  return "F005 - Flame disappears during operation";
    case 6:  return "F006 - Flame simulation error";
    case 7:  return "F007 - No or insufficient ionisation flow";
    case 8:  return "F008 - Fan speed incorrect";
    case 9:  return "F009 - Burner controller has internal fault";
    case 10: return "F010 - Sensor fault";
    case 11: return "F011 - Sensor fault";
    case 12: return "F012 - Sensor 5 fault";
    case 14: return "F014 - Mounting fault sensor";
    case 15: return "F015 - Mounting fault sensor S1";
    case 16: return "F016 - Mounting fault S3";
    case 18: return "F018 - Flue and/or air supply duct is blocked";
    case 19: return "F019 - BMM error";
    case 27: return "F027 - Short circuit of outdoor";
    case 28: return "F028 - Reset error";
    case 29: return "F029 - Gas valve error";
    case 30: return "F030 - Sensor S3 fault";
    case 31: return "F031 - Sensor fault S1";
    case 0xff: return "No Fault detected";
    default: return "Unspecified fault: " + esphome::to_string(u_code);
    }
}

}  // namespace intergas_xtreme_monitor_component
}  // namespace esphome
