#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_FUEL_FLOW

#include <dronecan_msgs.h>

void AP_Periph_FW::can_fuel_flow_update(void)
{
    if (!fuel_flow.enabled()) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - last_fuel_flow_update_ms < fuel_flow.get_rate() ){
        return;
    }
    last_fuel_flow_update_ms = now;
    fuel_flow.update();
    
    if (fuel_flow.option_is_set(AP_Senserion_FuelFlow::Options::FuelTankStatus)) {

        uavcan_equipment_ice_FuelTankStatus pkt {};

        pkt.fuel_consumption_rate_cm3pm = fuel_flow.get_flow();
        pkt.fuel_temperature = fuel_flow.get_temp();
        pkt.fuel_tank_id = fuel_flow.get_id();
        pkt.available_fuel_volume_cm3 = fuel_flow.get_estimated_consumed_fuel();

        uint8_t buffer[UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_MAX_SIZE] {};
        const uint16_t total_size = uavcan_equipment_ice_FuelTankStatus_encode(&pkt, buffer, !periph.canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
        }

    if (fuel_flow.option_is_set(AP_Senserion_FuelFlow::Options::ReciprocatingStatus)){

        uavcan_equipment_ice_reciprocating_Status pkt {};

        pkt.fuel_consumption_rate_cm3pm = fuel_flow.get_flow();
        pkt.oil_temperature = fuel_flow.get_temp();
        pkt.estimated_consumed_fuel_volume_cm3 = fuel_flow.get_estimated_consumed_fuel();

        if (fuel_flow.is_air_in_flow()) {
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_DETECTED;

        }else{

            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_SUPPORTED;
        }

        if(fuel_flow.is_high_flow()){
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_ABOVE_NOMINAL;
        }else{
            pkt.flags |= UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_SUPPORTED;
        }
        
        uint8_t buffer[UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_MAX_SIZE] {};
        const uint16_t total_size = uavcan_equipment_ice_reciprocating_Status_encode(&pkt, buffer, !canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_SIGNATURE,
                        UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }



}

#endif // HAL_PERIPH_ENABLE_FUEL_FLOW