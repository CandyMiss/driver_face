#ifndef DRIVER_MONITOR_COMMON_H
#define DRIVER_MONITOR_COMMON_H

#include <fstream>
#include <map>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <NvInfer.h>
#include "yololayer.h"

using namespace nvinfer1;

namespace YoloV5
{
    cv::Mat preprocess_img(cv::Mat &img);
    cv::Point get_point(cv::Mat &img, cv::Point& point);
    cv::Rect get_rect(cv::Mat &img, float bbox[4]);
    float iou(float lbox[4], float rbox[4]);
    bool cmp(Yolo::Detection &a, Yolo::Detection &b);
    void nms(std::vector<Yolo::Detection> &res, float *output, float conf_thresh, float nms_thresh = 0.5);
    std::map<std::string, Weights> loadWeights(const std::string file);
    IScaleLayer *addBatchNorm2d(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input,
                                std::string lname, float eps);
    ILayer *convBnLeaky(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int outch,
                        int ksize, int s, int g, std::string lname);
    ILayer *focus(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int inch, int outch,
                  int ksize, std::string lname);
    ILayer *bottleneck(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int c1, int c2,
                       bool shortcut, int g, float e, std::string lname);
    ILayer *bottleneckCSP(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int c1, int c2,
                          int n, bool shortcut, int g, float e, std::string lname);
    ILayer *SPP(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int c1, int c2, int k1,
                int k2, int k3, std::string lname);
    int read_files_in_dir(const char *p_dir_name, std::vector<std::string> &file_names);
}


#endif //DRIVER_MONITOR_COMMON_H
