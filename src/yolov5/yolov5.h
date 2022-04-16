#ifndef DRIVER_MONITOR_YOLOV5_H
#define DRIVER_MONITOR_YOLOV5_H

#include <iostream>
#include <chrono>
#include "cuda_runtime_api.h"
#include "../utils/logging.h"
#include "common.h"

#define USE_FP16  // comment out this if want to use FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

#define NET s  // s m l x
#define NETSTRUCT(str) createEngine_##str
#define CREATENET(net) NETSTRUCT(net)
#define STR1(x) #x
#define STR2(x) STR1(x)

namespace YoloV5
{
    const std::string ENGIN_PATH = "/home/nvidia/wdq/YoloGenEngine/build/";
    
    REGISTER_TENSORRT_PLUGIN(YoloPluginCreator);

    void doInference(IExecutionContext &context, float *input, float *output, int batchSize);

    void InitYoloV5Engine();

    void ReleaseYoloV5Engine();

    std::vector<Yolo::Detection> AnalyzeOneShot(cv::Mat &frame);

    std::vector<std::vector<Yolo::Detection>> AnalyzeBatch(std::vector<cv::Mat> &frames);

    void DrawYoloOutput(cv::Mat& frame, std::vector<Yolo::Detection> result);
}

#endif //DRIVER_MONITOR_YOLOV5_H
