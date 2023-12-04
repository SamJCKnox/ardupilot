// #pragma once

// #include "AP_TemperatureSensor_config.h"

// #if AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED

// #include<AP_DroneCAN/AP_DroneCAN.h>

// #define CAN_TEMPERATURE_TIMEOUT_MS 5000000

// class AP_TemperatureSensor_DroneCAN : public AP_TemperatureSensor
// {
// public:
//     AP_TemperatureSensor_DroneCAN(AP_HAL_Can* can, uint8_t node_id, uint8_t sensor_id) :
//         _can(can),
//         _node_id(node_id),
//         _sensor_id(sensor_id)
//     {}
    
//     static const struct AP_Param::GroupInfo var_info[];

//     bool init() override
//     {
//         return true;
//     }


//     bool read() override
//     {
//         // Send request for temperature data
//         uint8_t request[2] = {_node_id, _sensor_id};
//         _can->send(CAN_MESSAGE_TEMPERATURE_REQUEST, request, sizeof(request));

//         // Wait for response
//         uint32_t start_time = AP_HAL::millis();
//         while (AP_HAL::millis() - start_time < CAN_TEMPERATURE_TIMEOUT_MS) {
//             if (_can->available()) {
//                 uint8_t msg_id;
//                 uint8_t response[2];
//                 if (_can->receive(&msg_id, response, sizeof(response)) &&
//                     msg_id == CAN_MESSAGE_TEMPERATURE_RESPONSE &&
//                     response[0] == _node_id &&
//                     response[1] == _sensor_id)
//                 {
//                     // Temperature data received
//                     _temperature = response[2];
//                     return true;
//                 }
//             }
//         }

//         // Timeout waiting for response
//         return false;
//     }
//     static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    

// private:
//     AP_HAL_Can* _can;
//     uint8_t _node_id;
//     uint8_t _sensor_id;
// };
// #endif // HAL_ENABLE_DRONECAN_DRIVER