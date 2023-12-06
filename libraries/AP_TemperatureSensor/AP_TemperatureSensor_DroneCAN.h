#pragma once

#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED

#include "AP_TemperatureSensor_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>


class AP_TemperatureSensor_DroneCAN : public AP_TemperatureSensor_Backend
{   
public:

    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;
    
    static const struct AP_Param::GroupInfo var_info[];

    void init(void) override;

    void update(void) override {};

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);


private:
    float _temperature; // Celsius

    static void handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_device_Temperature &msg);

    static AP_TemperatureSensor_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    // Module Detection Registry
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;
        uint8_t node_id;
        AP_TemperatureSensor_DroneCAN *driver;
    } _detected_modules[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];
    
    static HAL_Semaphore _sem_registry;
    
    HAL_Semaphore _sem_temperature;
};
#endif // HAL_ENABLE_DRONECAN_DRIVER