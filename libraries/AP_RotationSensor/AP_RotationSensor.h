#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

class AP_RotationSensor
{
public:
    AP_RotationSensor();
    bool init();
    bool enabled();
    void update();
    float aoa_rads();
    float aos_rads();
    int battery_address();
    int get_update_rate();

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum Mode {INIT, PROGRAMMING_START, PROGRAMMING_WAIT, PROGRAMMING_FINALISE, PROGRAMMING_CHECK, STARTUP, MEASURE, ERR};
    Mode mode = Mode::INIT;

    bool probe(uint8_t address);
    bool burn_start();
    bool burn_wait();
    bool burn_finalise();
    bool burn_check();
    bool measure_primary();
    bool measure_secondary();
    bool convert_and_assign();

    float aoa;
    float aos;

    uint32_t init_time;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> primary;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> secondary;

    AP_Float aoa_trim;
    AP_Float aos_trim;

    uint16_t primary_raw;
    uint16_t secondary_raw;

    AP_Int8 enable;
    AP_Int8 program_enable;
    AP_Int8 bus;
    AP_Int8 aoa_is_secondary_address;
    AP_Int8 aoa_dir;
    AP_Int8 aos_dir;
    AP_Int8 send_battery;
    AP_Int8 update_rate;
};