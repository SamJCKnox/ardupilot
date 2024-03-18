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
    float get_estimated_consumed_fuel();
    int8_t get_id();
    bool is_air_in_flow();
    bool is_high_flow();

    static const struct AP_Param::GroupInfo var_info[];


    enum class Options : uint8_t {
        FuelTankStatus                   = (1U<<0),  
        ReciprocatingStatus              = (1U<<1),
    };

        // check if a option is set
    bool option_is_set(const AP_Senserion_FuelFlow::Options option) const {
        return (uint8_t(msg_type) & uint8_t(option)) != 0;
    }

private:

    bool measure();
    void convert_and_assign();

    float flow;
    float estimated_consumed_fuel;
    float temp;
    bool air_in_flow;
    bool high_flow;
    Options options;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> sensor;

    int64_t flow_raw; 
    uint16_t temp_raw; 
    uint16_t flags_raw;
    uint32_t last_updated_ms;
    uint32_t now;

    AP_Int8 invFlowScaleFactor; 
    AP_Int8 enable;
    AP_Int8 bus;
    AP_Float flow_offset;
    AP_Float flow_slope;
    AP_Int8 id;
    AP_Int8 msg_type;
};