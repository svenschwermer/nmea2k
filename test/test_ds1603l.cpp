#include "ds1603l.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono_literals;

int main()
{
    Stream stream;
    DS1603L uut(stream);

    cout << "Starting test\n";

#if 1
    stream.data.push(0xff);
    stream.data.push(0x01);
    stream.data.push(0x02);
    stream.data.push(0x02);
#else
    this_thread::sleep_for(11s);
#endif

    uint16_t reading = 0;
    auto state = uut.read(reading);
    cout << "STATUS: " << static_cast<int>(state) << "\n";
    cout << "READING: 0x" << hex << reading << "\n";

    cout << "Test done\n";

    return 0;
}

bool Stream::available()
{
    return !data.empty();
}

char Stream::read()
{
    auto c = data.front();
    data.pop();
    return c;
}
