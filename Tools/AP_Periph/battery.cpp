#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_BATTERY

        pkt.voltage = battery_lib.voltage(i);

        float current;
        if (battery_lib.current_amps(current, i)) {
            pkt.current = current;
        }
        float temperature;
        if (battery_lib.get_temperature(temperature, i)) {
            // Battery lib reports temperature in Celsius.
            // Convert Celsius to Kelvin for transmission on CAN.
            pkt.temperature = C_TO_KELVIN(temperature);
        }

        pkt.state_of_health_pct = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN;
        uint8_t percentage = 0;
        if (battery_lib.capacity_remaining_pct(percentage, i)) {
            pkt.state_of_charge_pct = percentage;
        }
        pkt.model_instance_id = i+1;

#if !defined(HAL_PERIPH_BATTERY_SKIP_NAME)
        // example model_name: "org.ardupilot.ap_periph SN 123"
        hal.util->snprintf((char*)pkt.model_name.data, sizeof(pkt.model_name.data), "%s %ld", AP_PERIPH_BATTERY_MODEL_NAME, (long int)serial_number);
        pkt.model_name.len = strnlen((char*)pkt.model_name.data, sizeof(pkt.model_name.data));
#endif //defined(HAL_PERIPH_BATTERY_SKIP_NAME)

        uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE] {};
        const uint16_t total_size = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);

        // Send individual cell information if available
        if (battery_lib.has_cell_voltages(i)) {
            can_battery_send_cells(i);
        }
    }
}

/*
  send individual cell voltages if available
 */
void AP_Periph_FW::can_battery_send_cells(uint8_t instance)
{
    // allocate space for the packet. This is a large
    // packet that won't fit on the stack, so dynamically allocate
    auto* pkt = new ardupilot_equipment_power_BatteryInfoAux;
    uint8_t* buffer = new uint8_t[ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_MAX_SIZE];
    if (pkt == nullptr || buffer == nullptr) {
        delete pkt;
        delete [] buffer;
        return;
    }
    const auto &cell_voltages = battery_lib.get_cell_voltages(instance);
			
    for (uint8_t i = 0; i < ARRAY_SIZE(cell_voltages.cells); i++) {
        if (cell_voltages.cells[i] == 0xFFFFU) {
            break;
        }
        pkt->voltage_cell.data[i] = cell_voltages.cells[i]*0.001;
        pkt->voltage_cell.len = i+1;
    }
			
    pkt->max_current = nanf("");
    pkt->nominal_voltage = nanf("");

    // encode and send message:
    const uint16_t total_size = ardupilot_equipment_power_BatteryInfoAux_encode(pkt, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_SIGNATURE,
                     ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     buffer,
                     total_size);

    // Delete temporary buffers
    delete pkt;
    delete [] buffer;
}

#endif // HAL_PERIPH_ENABLE_BATTERY

