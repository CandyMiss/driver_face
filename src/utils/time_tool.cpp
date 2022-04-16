#include "time_tool.h"

uint64_t now_to_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
}

TimeStamp GetTimeStamp(uint64_t ms)
{
    uint64_t mill = ms % 1000;  // 毫秒
    time_t tt = ms / 1000;  // 总秒数
    struct tm *local_time = localtime(&tt);

    TimeStamp stamp(*local_time, mill);

    return stamp;
}

TimeStamp GetNowStamp()
{
    return GetTimeStamp(now_to_ms());
}

