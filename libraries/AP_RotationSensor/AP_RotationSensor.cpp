#include "AP_RotationSensor.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

// Hacky but we need to be able to do this without including AP_Periph.h
extern "C"
{
    void can_printf(const char *fmt, ...) FMT_PRINTF(1, 2);
}

#define READ_ERROR UINT16_MAX

extern const AP_HAL::HAL &hal;

const uint8_t primary_address = 0x40;
const uint8_t secondary_address = 0x41;

const AP_Param::GroupInfo AP_RotationSensor::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Rotation Sensor Enable
    // @Description: Enable rotation sensor support
    // @Values: 0:Disable, 1:Enable
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 30, AP_RotationSensor, enable, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPINFO("_BURN_ADDR", 1, AP_RotationSensor, program_enable, 0),
    AP_GROUPINFO("_I2C_BUS", 2, AP_RotationSensor, bus, 0),
    AP_GROUPINFO("_AOA_ALT_ADDR", 4, AP_RotationSensor, aoa_is_secondary_address, 1),

    AP_GROUPINFO("_AOA_TRIM", 5, AP_RotationSensor, aoa_trim, 0),
    AP_GROUPINFO("_AOA_DIR", 6, AP_RotationSensor, aoa_dir, 0),

    AP_GROUPINFO("_AOS_TRIM", 7, AP_RotationSensor, aos_trim, 0),
    AP_GROUPINFO("_AOS_DIR", 8, AP_RotationSensor, aos_dir, 0),
    
    AP_GROUPINFO("_BATT_ID", 9, AP_RotationSensor, send_battery, 0),
    AP_GROUPINFO("_UPDATE_PERIOD_MS", 10, AP_RotationSensor, update_rate, 50),

    AP_GROUPEND};

AP_RotationSensor::AP_RotationSensor()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_RotationSensor::enabled()
{
    return enable;
}

bool AP_RotationSensor::probe(uint8_t address)
{
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = hal.i2c_mgr->get_device(bus, address);

    if (!dev)
    {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    uint8_t send = 0x20;
    uint8_t receive = 0;

    if (!dev->transfer(&send, sizeof(send), &receive, sizeof(receive)))
    {
        return false;
    }

    if ((receive >> 1) != address)
    {
        printf("unexpected address register");
        return false;
    }

    // configure PWM output
    uint8_t send_buf[] = {0x08, 0b00110000};

    if (!dev->transfer(send_buf, sizeof(send_buf), nullptr, 0))
    {
        printf("failed configuration\n");
        return false;
    }

    return true;
}

bool AP_RotationSensor::burn_start()
{
    if (AP_HAL::millis() - init_time < 2000)
    {
        // wait 2 seconds for the bus to be initialised before printing
        return false;
    }

    printf("Alt address program\n");
    can_printf("Alt address programming\n");

    if (probe(primary_address))
    {
        can_printf("Device already present at default 0x40, aborting");
        can_printf("Disconnect all devices before running this tool");

        mode = ERR;
        return false;
    }
    else
    {
        can_printf("Plug in sensor board now");
        mode = PROGRAMMING_WAIT;
        return true;
    }
}

bool AP_RotationSensor::burn_wait()
{
    if (probe(primary_address))
    {
        mode = PROGRAMMING_FINALISE;
        can_printf("Sensor found, programming to alt address");
        return true;
    }

    return false;
}

bool AP_RotationSensor::burn_finalise()
{
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = hal.i2c_mgr->get_device(bus, primary_address);

    WITH_SEMAPHORE(dev->get_semaphore());

    // Send the new address
    uint8_t addr_buf[] = {0x20, 0x41 << 1, 0x41 << 1};

    if (!dev->transfer(addr_buf, sizeof(addr_buf), nullptr, 0))
    {
        printf("failed configuration\n");
        mode = ERR;
        return false;
    }

    dev->set_address(secondary_address);

    uint8_t burn_buf[] = {0xFF, 0x40};

    if (!dev->transfer(burn_buf, sizeof(burn_buf), nullptr, 0))
    {
        printf("failed burn configuration\n");
        mode = ERR;
        return false;
    }

    mode = PROGRAMMING_CHECK;
    return true;
}

bool AP_RotationSensor::burn_check()
{
    if (probe(secondary_address))
    {
        can_printf("Programming successful");
        mode = MEASURE;
        return true;
    }
    else
    {
        can_printf("Programming error");
        mode = ERR;
        return true;
    }
}

bool AP_RotationSensor::measure_primary()
{
    if (!primary)
    {
        primary = hal.i2c_mgr->get_device(bus, primary_address);
        primary_raw = READ_ERROR; // use this as an error flag; the sensor is 12 bit

        return false;
    }

    WITH_SEMAPHORE(primary->get_semaphore());

    uint8_t send = 0x0C;
    uint8_t receive[2] = {0};

    if (!primary->transfer(&send, sizeof(send), receive, sizeof(receive)))
    {
        primary_raw = READ_ERROR;
        return false;
    }

    primary_raw = receive[0] << 8 | receive[1];

    return true;
}

bool AP_RotationSensor::measure_secondary()
{
    if (!secondary)
    {
        secondary = hal.i2c_mgr->get_device(bus, secondary_address);
        secondary_raw = READ_ERROR; // use this as an error flag; the sensor is 12 bit

        return false;
    }

    WITH_SEMAPHORE(secondary->get_semaphore());

    uint8_t send = 0x0C;
    uint8_t receive[2] = {0};

    if (!secondary->transfer(&send, sizeof(send), receive, sizeof(receive)))
    {
        secondary_raw = READ_ERROR;
        return false;
    }

    secondary_raw = receive[0] << 8 | receive[1];

    return true;
}

bool AP_RotationSensor::convert_and_assign()
{
    float primary_f = NAN;
    float secondary_f = NAN;

    if (primary_raw != READ_ERROR)
    {
        primary_f = (((float)primary_raw) / 4096.0f) * M_PI * 2;
        primary_f = wrap_2PI(primary_f);
    }

    if (secondary_raw != READ_ERROR)
    {
        secondary_f = (((float)secondary_raw) / 4096.0f) * M_PI * 2;
        secondary_f = wrap_2PI(secondary_f);
    }

    if (aoa_is_secondary_address)
    {
        aoa = aoa_dir ? secondary_f : (M_PI * 2) - secondary_f;
        aos = aos_dir ? primary_f : (M_PI * 2) - primary_f;

        aoa = wrap_2PI(aoa + aoa_trim) - M_PI;
        aos = wrap_2PI(aos + aos_trim) - M_PI;
    }
    else
    {
        aoa = aoa_dir ? primary_f : (M_PI * 2) - primary_f;
        aos = aos_dir ? secondary_f : (M_PI * 2) - secondary_f;

        aoa = wrap_2PI(primary_f + aoa_trim) - M_PI;
        aos = wrap_2PI(secondary_f + aos_trim) - M_PI;
    }

    return true;
}

float AP_RotationSensor::aoa_rads()
{
    return aoa;
}

float AP_RotationSensor::aos_rads()
{
    return aos;
}

int AP_RotationSensor::battery_address()
{
    return send_battery;
}
int AP_RotationSensor::get_update_rate()
{
    return update_rate;
}

void AP_RotationSensor::update()
{
    switch (mode)
    {
    case PROGRAMMING_START:
        burn_start();
        break;
    case PROGRAMMING_WAIT:
        burn_wait();
        break;
    case PROGRAMMING_FINALISE:
        burn_finalise();
        break;
    case PROGRAMMING_CHECK:
        burn_check();
        break;
    case MEASURE:
        measure_primary();
        measure_secondary();
        convert_and_assign();
        break;
    default:
        break;
    }
}

bool AP_RotationSensor::init()
{
    if (!enable)
    {
        return false;
    }

    if (program_enable)
    {
        mode = PROGRAMMING_START;

        init_time = AP_HAL::millis();

        AP_Param::set_and_save_by_name("FAD_BURN_ADDR", 0.0f);
    }
    else
    {
        mode = MEASURE;
    }

    return true;
}