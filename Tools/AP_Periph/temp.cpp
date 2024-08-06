#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_TEMP

#include <dronecan_msgs.h>


/*
  update CAN temp
 */
void AP_Periph_FW::can_temp_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_temp_update_ms < 5000) {
        // max 0.2Hz data
        return;
    }
    last_temp_update_ms = now;

    temperature_sensor.update();


    const uint8_t temp_instances = temperature_sensor.num_instances();
    for (uint8_t i=0; i<temp_instances; i++) {

    if (!temperature_sensor.healthy(i)) {
        // don't send any data
        return;
    }

    float temp;
    if(!temperature_sensor.get_temperature(temp, i)){
        return;
    }

    uavcan_protocol_debug_KeyValue pkt{};
    pkt.value = temp;
    //pkt.key = temperature_sensor.get_source_id(i);
    pkt.key.data[0] = temperature_sensor.get_source_id(i);
    pkt.key.len = 1;

    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE] {};
    uint16_t total_size = uavcan_protocol_debug_KeyValue_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
                UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
                CANARD_TRANSFER_PRIORITY_LOW,
                &buffer[0],
                total_size);

    // uavcan_equipment_device_Temperature pkt {};

    // pkt.temperature = temp;
    // pkt.device_id = temperature_sensor.get_source_id(i);


    // uint8_t buffer[UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE] {};
    // uint16_t total_size = uavcan_equipment_device_Temperature_encode(&pkt, buffer, !periph.canfdout());

    // canard_broadcast(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE,
    //                 UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID,
    //                 CANARD_TRANSFER_PRIORITY_LOW,
    //                 &buffer[0],
    //                 total_size);
}
}

#endif // HAL_PERIPH_ENABLE_TEMP
