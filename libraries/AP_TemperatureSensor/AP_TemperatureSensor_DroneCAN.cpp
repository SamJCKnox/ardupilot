#include "AP_TemperatureSensor_DroneCAN.h"

#if AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "Temp"

AP_TemperatureSensor_DroneCAN::DetectedModules AP_TemperatureSensor_DroneCAN::_detected_modules[];

HAL_Semaphore AP_TemperatureSensor_DroneCAN::_sem_registry;

void AP_TemperatureSensor_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_temperature, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("temperature_sensor_sub");
    }
}

void AP_TemperatureSensor_DroneCAN::init()
{
    // always returns true
    return;

}

AP_TemperatureSensor_DroneCAN* AP_TemperatureSensor_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AP_TEMPERATURE_SENSOR_MAX_INSTANCES; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < AP_TEMPERATURE_SENSOR_MAX_INSTANCES; i++) {
        if (_detected_modules[i].ap_dronecan == ap_dronecan && _detected_modules[i].node_id == node_id) {
            // detected
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < AP_TEMPERATURE_SENSOR_MAX_INSTANCES; i++) {
            if (_detected_modules[i].ap_dronecan == nullptr) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_TemperatureSensor_DroneCAN::handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_device_Temperature &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_TemperatureSensor_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_temperature);
        driver->_temperature = KELVIN_TO_C(msg.temperature);
    }
}


#endif // AP_BATTERY_UAVCAN_TEMP_ENABLED
