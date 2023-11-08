#include "AP_Periph.h"

#ifdef AP_TEMPERATURE_SENSOR_ENABLED

#include <dronecan_msgs.h>


/*
  update CAN temp
 */
void AP_Periph_FW::can_temp_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_temp_update_ms < 50) {
        // max 20Hz data
        return;
    }
    last_temp_update_ms = now;
    temperature_sensor.update();
    if (!temperature_sensor.healthy()) {
        // don't send any data
        return;
    }
    float temp;
    if(!temperature_sensor.get_temperature(temp)){
        return;
    }

    uavcan_equipment_device_Temperature pkt {};

    // unfilled elements are NaN
    pkt.temperature = nanf("");


    // populate the elements we have
    pkt.temperature = temp;


    uint8_t buffer[UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE] {};
    uint16_t total_size = uavcan_equipment_device_Temperature_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE,
                    UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}

#endif // AP_TEMPERATURE_SENSOR_ENABLED
