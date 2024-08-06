#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_MPRLS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_MPRLS_I2C_ADDR
#define HAL_BARO_MPRLS_I2C_ADDR        (0x18)
#endif

class AP_Baro_MPRLS : public AP_Baro_Backend
{
public:

    AP_Baro_MPRLS(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:

    bool _init();
    void _timer(void);
    void _update_pressure(uint32_t raw_press);
    void _update_temperature(uint32_t raw_temp);

    uint8_t _instance;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;

    uint32_t _output_min = 1677722;
    uint32_t _output_max = 15099494;
    uint8_t _psi_min = 0;
    uint8_t _psi_max = 25;


    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
};

#endif 
