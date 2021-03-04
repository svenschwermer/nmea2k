#ifndef _DS1603L_H_
#define _DS1603L_H_

#include <Arduino.h>
#include <Stream.h>

class DS1603L {
  public:
    enum class state {
        new_data,
        no_new_data,
        checksum_fail,
        no_input,
    };

    DS1603L(Stream &serial)
        : serial(serial), buffer(0), last_data_time(millis()) {}
    state read(uint16_t &reading);

  private:
    Stream &serial;
    uint32_t buffer;
    uint32_t last_data_time;
};

#endif
