#include "ds1603l.h"

DS1603L::state DS1603L::read(uint16_t &reading)
{
    uint32_t old_time = last_data_time;

    while (serial.available())
    {
        last_data_time = millis();
        uint8_t byte = serial.read();
        buffer = (buffer << 8) | byte;

        if (buffer & 0xff000000 == 0xff000000)
        {
            uint8_t checksum = 0xff;
            checksum += (buffer >> 16) & 0xff;
            checksum += (buffer >> 8) & 0xff;
            if (checksum == byte)
            {
                reading = (buffer >> 8) & 0xffff;
                return state::new_data;
            }
            return state::checksum_fail;
        }
    }
    if (old_time == last_data_time && millis() - last_data_time > 10000)
        return state::no_input;
    return state::no_new_data;    
}
