#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED
#include "AP_TemperatureSensor.h"
#include "AP_TemperatureSensor_DroneCAN.h"
#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define LOG_TAG "Temp"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_TemperatureSensor_DroneCAN::var_info[] = {

    // @Param: ID
    // @DisplayName: ID of temperature sensor
    // @Description: ID used to match temperature sensor to a specific node
    // @Range: 1 127
    // @User: Advanced
    AP_GROUPINFO("ID", 30, AP_TemperatureSensor_DroneCAN, node_id, 1),

    // Param indexes must be between 30 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

//constructor
AP_TemperatureSensor_DroneCAN::AP_TemperatureSensor_DroneCAN(AP_TemperatureSensor &temp, AP_TemperatureSensor::TemperatureSensor_State &temp_state, AP_TemperatureSensor_Params &params) :
    AP_TemperatureSensor_Backend(temp, temp_state, params)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

}

void AP_TemperatureSensor_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{

}

void AP_TemperatureSensor_DroneCAN::init()
{
    // always returns true
    return;

}

void AP_TemperatureSensor_DroneCAN::handle_temperature(const CanardRxTransfer& transfer)
{

}


#endif // AP_BATTERY_UAVCAN_TEMP_ENABLED
