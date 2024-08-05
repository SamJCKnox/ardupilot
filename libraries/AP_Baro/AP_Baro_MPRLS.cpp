/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Baro_MPRLS.h"

#if AP_BARO_MPRLS_ENABLED

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

#define MPRLS_BUF_ADDR          (0xAA)
#define MPRLS_READ_TIMEOUT      (20)     
#define MPRLS_STATUS_POWERED    (0x40) 
#define MPRLS_STATUS_BUSY       (0x20)    
#define MPRLS_STATUS_FAILED     (0x04)  
#define MPRLS_STATUS_MATHSAT    (0x01) 
#define COUNTS_224              (16777216L)      
#define PSI_to_PA               (6894.7572932)   
#define MPRLS_STATUS_MASK       (0b01100101) 

#define OUTPUT_MIN              (uint32_t)((float)COUNTS_224 * (10 / 100.0) + 0.5);
#define OUTPUT_MAX              (uint32_t)((float)COUNTS_224 * (90 / 100.0) + 0.5);
#define PSI_MIN                 (0)
#define PSI_MAX                 (25)

AP_Baro_MPRLS::AP_Baro_MPRLS(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_MPRLS::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev)
{
    if (!_dev) {
        return nullptr;
    }

    AP_Baro_MPRLS *sensor = new AP_Baro_MPRLS(baro, std::move(_dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_MPRLS::_init()
{
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->setup_checked_registers(2, 20);

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_MPRLS);
    set_bus_id(_instance, _dev->get_bus_id());
    
    // request 50Hz update
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_MPRLS::_timer, void));

    return true;
}


//  accumulate a new sensor reading
void AP_Baro_MPRLS::_timer(void)
{
    uint8_t buf[7];

    _dev->read_registers(MPRLS_BUF_ADDR, buf, sizeof(buf));
      // check status byte
    if (buf[0] & MPRLS_STATUS_MATHSAT) {
        return;
    }
    if (buf[0] & MPRLS_STATUS_FAILED) {
        return;
    }

    _update_pressure(uint32_t(buf[1]) << 16) | (uint32_t(buf[2]) << 8) |
         (uint32_t(buf[3]));
    _update_temperature(uint32_t(buf[4]) << 16) | (uint32_t(buf[5]) << 8) |
         (uint32_t(buf[6]));

    _dev->check_next_register();
}

void AP_Baro_MPRLS::_update_pressure(uint32 raw_press){
        if (raw_press == 0xFFFFFFFF ||  OUTPUT_MIN == OUTPUT_MAX) {
            return;
        }
    float raw_psi =  (raw_press / 16777215) * 100;

    // All is good, calculate and convert to desired units using provided factor
    // use the 10-90 calibration curve by default or whatever provided by the user
    float psi = (raw_psi - OUTPUT_MIN) * (PSI_MAX - PSI_MIN);
    psi /= (float)(OUTPUT_MAX - OUTPUT_MIN);
    psi += PSI_MIN;
    // convert to desired units
    _pressure_sum += psi * PSI_to_HPA;
    _pressure_count++;
}

void AP_Baro_MPRLS::_update_temperature(uint32_t raw_temp)
{
    WITH_SEMAPHORE(_sem);
    _temperature = (raw_temp * 200 / 16777215) - 50 - 273.15;
}

// transfer data to the frontend
void AP_Baro_MPRLS::update(void)
{
    WITH_SEMAPHORE(_sem);

    _copy_to_frontend(_instance,
                      _pressure_sum/_pressure_count,
                      _temperature);

    _pressure_sum = 0;
    _pressure_count = 0;
}

#endif  
