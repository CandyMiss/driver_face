#ifndef DRIVER_MONITOR_GST_DECODE_H
#define DRIVER_MONITOR_GST_DECODE_H

#include <iostream>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <mutex>
#include <chrono>
#include <queue>

using namespace std;

class GstCameraPipeline
{
public:
    static const int CAPTURE_WIDTH = 1920;
    static const int CAPTURE_HEIGHT = 1080;
    static const int DISPLAY_WIDTH = 1280;
    static const int DISPLAY_HEIGHT = 720;
    static const int FRAMERATE = 30;
    static const int FLIP_MODE = 0;

    static const int DECODE_264 = 0;
    static const int DECODE_265 = 1;

    // 需要了解硬件传过来的具体数据格式：x-raw、x-h264还是x-h265？但是交给opencv的格式确定就是video/x-raw, format=(string)BGR
    static std::string GetPipelineUSB(int devIndex)
    {
        return "v4l2src device=/dev/video" + std::to_string(devIndex) + " ! video/x-raw,width=" +
               std::to_string(DISPLAY_WIDTH) + ", height=" + std::to_string(DISPLAY_HEIGHT) + ", framerate=" +
               std::to_string(FRAMERATE) + "/1 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    static std::string GetPipelineCSI(int sensor_id)
    {
        return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " ! video/x-raw(memory:NVMM), width=(int)" +
               std::to_string(CAPTURE_WIDTH) + ", height=(int)" + std::to_string(CAPTURE_HEIGHT) +
               ", format=(string)NV12, framerate=(fraction)" + std::to_string(FRAMERATE) +
               "/1 ! nvvidconv flip-method=" + std::to_string(FLIP_MODE) + " ! video/x-raw, width=(int)" +
               std::to_string(DISPLAY_WIDTH) + ", height=(int)" + std::to_string(DISPLAY_HEIGHT) +
               ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    static std::string GetPipelineRTSP(string str_rtsp, int decode_mode)
    {
        std::string str_decode = "h264";
        if (decode_mode == DECODE_265)
        {
            str_decode = "h265";
        }
        return "rtspsrc location=" + str_rtsp + " latency=200 ! rtp" + str_decode + "depay ! " + str_decode +
               "parse ! omx" + str_decode + "dec ! nvvidconv ! video/x-raw, width=" + std::to_string(DISPLAY_WIDTH) +
               ", height=" + std::to_string(DISPLAY_HEIGHT) +
               ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

};

#endif //DRIVER_MONITOR_GST_DECODE_H
