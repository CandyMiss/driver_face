#ifndef DRIVER_MONITOR_TIME_TOOL_H
#define DRIVER_MONITOR_TIME_TOOL_H

#include<chrono>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>

uint64_t now_to_ms();

class TimeStamp
{
public:
    int year;       /* Year	- 1900.  */
    int mon;        /* Month.	[0-11] */
    int day;       /* Day.		[1-31] */
    int hour;       /* Hours.	[0-23] */
    int min;        /* Minutes.	[0-59] */
    int sec;        /* Seconds.	[0-60] (1 leap second) */
    int milli_sec;  /* Milli Seconds.	[0-1000]  */

    TimeStamp()
    {
        uint64_t ms = now_to_ms();
        milli_sec = ms % 1000;  // 毫秒
        time_t tt = ms / 1000;  // 总秒数
        struct tm *local_time = localtime(&tt);

        year = local_time->tm_year;
        mon = local_time->tm_mon;
        day = local_time->tm_mday;
        hour = local_time->tm_hour;
        min = local_time->tm_min;
        sec = local_time->tm_sec;
    }

    TimeStamp(int year, int mon, int day, int hour, int min, int sec, int milli_sec) :
            year(year), mon(mon), day(day), hour(hour), min(min), sec(sec), milli_sec(milli_sec)
    {

    }

    TimeStamp(tm time, int milli_sec) :
            year(time.tm_year), mon(time.tm_mon), day(time.tm_mday), hour(time.tm_hour), min(time.tm_min),
            sec(time.tm_sec), milli_sec(milli_sec)
    {

    }

    bool operator<(const TimeStamp &d)
    {
        if (year < d.year)
        {
            return true;
        }
        else if (year == d.year)
        {
            if (mon < d.mon)
            {
                return true;
            }
            else if (mon == d.mon)
            {
                if (day < d.day)
                {
                    return true;
                }
                else if (day == d.day)
                {
                    if (hour < d.hour)
                    {
                        return true;
                    }
                    else if (hour == d.hour)
                    {
                        if (min < d.min)
                        {
                            return true;
                        }
                        else if (min == d.min)
                        {
                            if (sec < d.sec)
                            {
                                return true;
                            }
                            else if (sec == d.sec)
                            {
                                if (milli_sec < d.milli_sec)
                                {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    bool operator>(const TimeStamp &d)
    {
        if (year > d.year)
        {
            return true;
        }
        else if (year == d.year)
        {
            if (mon > d.mon)
            {
                return true;
            }
            else if (mon == d.mon)
            {
                if (day > d.day)
                {
                    return true;
                }
                else if (day == d.day)
                {
                    if (hour > d.hour)
                    {
                        return true;
                    }
                    else if (hour == d.hour)
                    {
                        if (min > d.min)
                        {
                            return true;
                        }
                        else if (min == d.min)
                        {
                            if (sec > d.sec)
                            {
                                return true;
                            }
                            else if (sec == d.sec)
                            {
                                if (milli_sec > d.milli_sec)
                                {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }


        return false;
    }

    bool operator==(const TimeStamp &d)
    {
        return (year == d.year &&
                mon == d.mon &&
                day == d.day &&
                hour == d.hour &&
                min == d.min &&
                sec == d.sec &&
                milli_sec == d.milli_sec);
    }

    bool operator<=(const TimeStamp &d)
    {
        return *this < d || *this == d;
    }

    bool operator>=(const TimeStamp &d)
    {
        return *this > d || *this == d;
    }

    // 转为官方时间
    std::chrono::system_clock::time_point toMillisecond()
    {
        char _time[25] = {0};
        sprintf(_time, "%d-%02d-%02d %02d:%02d:%02d", year, mon, day, hour, min, sec);

        std::string str(_time);
        time_t t_;
        tm tm_;
        strptime(str.c_str(), "%Y-%m-%d %H:%M:%S", &tm_); //将字符串转换为tm时间
        t_ = mktime(&tm_); //将tm时间转换为秒时间

        return std::chrono::system_clock::time_point(std::chrono::milliseconds(t_ * 1000 + milli_sec));
    }

    std::string toString()
    {
        std::stringstream strStream;
        strStream << year << "年" << mon << "月" << day << "日 " << hour << "时" << min << "分" << sec << "秒" << milli_sec
                  << "ms";

        return strStream.str();
    }

    std::string toStringDebug()
    {
        std::stringstream strStream;
        strStream << sec << "秒" << milli_sec << "ms";

        return strStream.str();
    }
};


TimeStamp GetTimeStamp(uint64_t ms);
TimeStamp GetNowStamp();

#endif //DRIVER_MONITOR_TIME_TOOL_H
