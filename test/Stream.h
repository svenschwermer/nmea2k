#ifndef _STREAM_H_
#define _STREAM_H_

#include <queue>

struct Stream
{
    bool available();
    char read();

    std::queue<char> data;
};

#endif
