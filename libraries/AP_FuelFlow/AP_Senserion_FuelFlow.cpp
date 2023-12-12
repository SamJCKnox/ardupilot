#include "AP_Senserion_FuelFlow.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

// From SENSIRIONI2CSF06LF_H
// INV_FLOW_SCALE_FACTORS_SLF3C_1300F = 500,
// INV_FLOW_SCALE_FACTORS_SLF3S_1300F = 500,
// INV_FLOW_SCALE_FACTORS_SLF3S_4000B = 32,
// INV_FLOW_SCALE_FACTORS_SLF3S_0600F = 10,
// INV_FLOW_SCALE_FACTORS_LD20_0600L = 1200,
// INV_FLOW_SCALE_FACTORS_LD20_2600B = 20,

// Hacky but we need to be able to do this without including AP_Periph.h
extern "C"
{
    void can_printf(const char *fmt, ...) FMT_PRINTF(1, 2);
}

extern const AP_HAL::HAL &hal;

const uint8_t sensor_address = 0x08;


const AP_Param::GroupInfo AP_Senserion_FuelFlow::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Rotation Sensor Enable
    // @Description: Enable rotation sensor support
    // @Values: 0:Disable, 1:Enable
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 30, AP_Senserion_FuelFlow, enable, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPINFO("_I2C_BUS", 2, AP_Senserion_FuelFlow, bus, 0),

    AP_GROUPINFO("_FLOW_SCALE_FACTOR", 3, AP_Senserion_FuelFlow, invFlowScaleFactor, 32),

    AP_GROUPINFO("_FLOW_CAL_SLOPE", 4, AP_Senserion_FuelFlow, flow_slope, 1),

    AP_GROUPINFO("_FLOW_CAL_OFFSET", 5, AP_Senserion_FuelFlow, flow_offset, 0),

    AP_GROUPEND};

AP_Senserion_FuelFlow::AP_Senserion_FuelFlow()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_Senserion_FuelFlow::enabled()
{
    return enable;
}

bool AP_Senserion_FuelFlow::probe()
{
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = hal.i2c_mgr->get_device(bus, sensor_address);

    if (!dev)
    {
        printf("Unable to make device\n");
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // Send stop command
    uint8_t send_buf[] = {0x3F, 0xF9};

    if (!dev->transfer(send_buf, sizeof(send_buf), nullptr, 0))
    {
        printf("failed configuration\n");
        return false;
    }

    // Send continuous command (IPA)
    uint8_t send_buf2[] = {0x36, 0x15};

    if (!dev->transfer(send_buf2, sizeof(send_buf2), nullptr, 0))
    {
        printf("failed configuration\n");
        return false;
    }

    return true;
}

bool AP_Senserion_FuelFlow::measure()
{
    if (!sensor)
    {
        sensor = hal.i2c_mgr->get_device(bus, sensor_address);

        return false;
    }

    WITH_SEMAPHORE(sensor->get_semaphore());

    uint8_t send = 0;
    uint8_t receive[9] = {0};

    if (!sensor->transfer(&send, sizeof(send), receive, sizeof(receive)))
    {
        return false;
    }

    flow_raw = static_cast<int16_t> (receive[0] << 8 | receive[1]);
    temp_raw = receive[3] << 8 | receive[4];
    flags_raw = receive[6] << 8 | receive[7];
    
    air_in_flow = (flags_raw & 0x01) == 0x01;
    high_flow = (flags_raw & 0x02) == 0x02;

    return true;
}


void AP_Senserion_FuelFlow::convert_and_assign()
{
    flow = 0.0;
    flow = (float)(flow_raw);
    flow = flow_slope * (flow / (int)(invFlowScaleFactor)) + flow_offset;

    temp = 0.0;
    temp = temp_raw / 200.0;
}

float AP_Senserion_FuelFlow::get_flow()
{
    return flow;
}

float AP_Senserion_FuelFlow::get_temp()
{
    return temp;
}

bool AP_Senserion_FuelFlow::is_air_in_flow()
{
    return air_in_flow;
}
bool AP_Senserion_FuelFlow::is_high_flow()
{
    return high_flow;
}

void AP_Senserion_FuelFlow::update()
{
    if(!measure()){
        printf("Measuring Error\n");
        return;
    }
    convert_and_assign();
}

bool AP_Senserion_FuelFlow::init()
{
    if (!enable)
    {
        return false;
    }

    if(!probe()){
        printf("Initialisation Error\n");
        return false;
    }
    return true;
}