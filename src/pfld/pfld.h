#ifndef DRIVER_MONITOR_PFLD_H
#define DRIVER_MONITOR_PFLD_H

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "../utils/utils.h"
#include "../utils/logging.h"

using namespace nvinfer1;

namespace PFLD
{
    static const int POINT_NUM = 98;
    static const int OUTPUT_SIZE = POINT_NUM * 2;

    void doInference(IExecutionContext &context, float *input, float *output, int batchSize);

    void InitPFLDEngine();

    void ReleasePFLDEngine();

    int AnalyzeOneFace(cv::Mat &frame, float* prob);

    void DrawFaceOutput(cv::Mat &frame, float *prob);
}

#endif //DRIVER_MONITOR_PFLD_H
