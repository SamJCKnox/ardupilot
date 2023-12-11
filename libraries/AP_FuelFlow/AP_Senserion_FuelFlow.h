#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>


class AP_Senserion_FuelFlow
{
public:
    AP_Senserion_FuelFlow();
    bool init();
    void update();
    bool enabled();
    float get_flow();
    float get_temp();
    bool is_air_in_flow();
    bool is_high_flow();

    static const struct AP_Param::GroupInfo var_info[];

private:

    bool probe();

    bool measure();

    void convert_and_assign();

    float flow;
    float temp;
    bool air_in_flow;
    bool high_flow;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> sensor;

    int64_t flow_raw; 
    uint16_t temp_raw; 
    uint16_t flags_raw;

    AP_Int8 enable;
    AP_Int8 bus;
};