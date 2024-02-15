#pragma once

#include "AP_TemperatureSensor.h"
#include "AP_TemperatureSensor_Backend.h"

#if AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>


class AP_TemperatureSensor_DroneCAN : public AP_TemperatureSensor_Backend {
        using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;
    
public:
    /// Constructor
    AP_TemperatureSensor_DroneCAN(AP_TemperatureSensor &temp, AP_TemperatureSensor::TemperatureSensor_State &temp_state, AP_TemperatureSensor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    void init(void) override;

    void update(void) override {};

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);


private:
    float _temperature; // Celsius

    AP_DroneCAN* ap_dronecan;
    AP_Int8 node_id;

    HAL_Semaphore _sem_temperature;

    void handle_temperature(const CanardRxTransfer&);
};
#endif // AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED