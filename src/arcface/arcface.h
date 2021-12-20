#ifndef DRIVER_MONITOR_ARCFACE_H
#define DRIVER_MONITOR_ARCFACE_H

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "../utils/logging.h"
#include "../utils/utils.h"

using namespace nvinfer1;

namespace ArcFace
{
    const static unsigned int FACE_FEATURE_DIMENSION = 512; //512维人脸特征
    const static unsigned int GALLARY_NUM = 2000;   //2000人

    static Logger gLogger;

    void ReadFaceDataToGPU();

    std::map<std::string, Weights> loadWeights(const std::string file);

    IScaleLayer *addBatchNorm2d(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, std::string lname, float eps);

    ILayer *addPRelu(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, std::string lname);

    ILayer *resUnit(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int num_filters, int s, bool dim_match,
                    std::string lname);;

    ICudaEngine *createEngine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt);

    void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream);

    void InitArcFaceEngine();   //这里有路径？

    void ReleasePFLDEngine();

    void GetFaceFeature(cv::Mat faceImg, float *faceFeature);

    void DetectFaceID(cv::Mat faceImg, int &faceID, float *faceBestFeature);

    void doInference(float *input, float *output);

    void doInferenceGetID(float *input, float *output, int &faceId);

    float GetSimilarOfTwoFace(float *faceCur, float *faceBest);
}

#endif //DRIVER_MONITOR_ARCFACE_H
