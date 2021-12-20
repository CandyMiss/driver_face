#ifndef DRIVER_MONITOR_RESULTS_H
#define DRIVER_MONITOR_RESULTS_H

#include <vector>
#include <map>

#include "../utils/time_tool.h"
// #include "VideoThread.h"
#include "../yolov5/yolov5.h"
#include "../utils/cfg_params.h"
#include "../utils/dataset.h"
#include "../pfld/pfld.h"
#include "../arcface/arcface.h"

#include <opencv2/core.hpp>

class DriverResult
{
public:
    // stage1：是否捕捉到头、脸、烟、手机、遮眼、遮嘴的状态
    bool HeadCaptured{false};
    bool FaceCaptured{false};  // ————非常重要的标志位
    bool FaceLeftCaptured{false};
    bool FaceRightCaptured{false};
    bool FaceUpCaptured{false};
    bool FaceDownCaptured{false};
    bool IsEyeOcclusion{false};
    bool IsMouthOcclusion{false};
    bool HasCigarette{false};
    bool HasPhone{false};

    // 捕捉到的数据
    float RectHead[Yolo::LOCATIONS]{0.0};    // Yolo::LOCATIONS 就是4。rect存储比值，center_x center_y w h
    float RectFace[Yolo::LOCATIONS]{0.0};   //人脸识别要用
    float RectFacePoint[Yolo::LOCATIONS]{0.0};    // 人脸关键点检测所用。它应当是rectFace之外更广阔一点的范围，需要包括双耳及额头
    float RectCigarette[Yolo::LOCATIONS]{0.0};
    float RectPhone[Yolo::LOCATIONS]{0.0};

    // 需要全部映射到yolo图像的位置，用比例
    float PointsFace[PFLD::OUTPUT_SIZE]{0.0}; // 98个面部关键点坐标数据

    // stage2: 计算头部转动、闭眼、张嘴的量化值。当面部关键点捕捉失败时，所有量化值为-1.0(正常状态下，这些量化值没有负数)

    bool IsFaceValid{true};    // 面部关键点抓取有可能失败！
    float FaceAngleHoriz{-1.0};
    float FaceAngelVirt{-1.0};
    float EyeOpen{-1.0};
    float MouthOpen{-1.0};

public:
    void DealYoloResult(std::vector<Yolo::Detection> &driverResult);

    void DealFaceResult(float *pointsFace);

    void ResetPointState();

    std::string toString();

private:
    void analyzeFaceState(float *pointsFace);

    void getFacePointMapPos(float *pointsFace);

};

class DriverIDResult
{
public:
    static const unsigned int FACE_RECOG_TIMES = 10;    //连续检测人脸的次数
    int ID{-1};
    float CurDriveFaceFeature[ArcFace::FACE_FEATURE_DIMENSION]{0.0};
    std::vector<std::string> DriverIds;

private:
    std::map<int, int> faceIDRec;
    std::map<int, float *> faceFeatureRec;

public:
    DriverIDResult();

    void Reset();

    void AddOneRecogResult(int faceID, float *faceFeature);

    void GetIDReuslt();

    std::string QueryDriverName();
};

#endif //DRIVER_MONITOR_RESULTS_H
