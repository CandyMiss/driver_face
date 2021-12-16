#ifndef DRIVER_MONITOR_CFG_PARAMS_H
#define DRIVER_MONITOR_CFG_PARAMS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <libxml/parser.h>
#include <libxml/xmlmemory.h>

struct FaceRecognitionConfig
{
    int RECOGNITION_TIME_PERIOD{10};
};

struct FaceOcclusionConfig
{
    int EYES_OCCLUSION_TIME_THRD{5};
    int MOUTH_OCCLUSION_TIME_THRD{5};
};

struct AbnormalBehaviorConfig
{
    int FACE_ANGLE_HORIZ_TIME_THRD{3};
    int FACE_ANGLE_VIRT_TIME_THRD{3};
    double ANGLE_HORIZ_THRD{0.2};
    double ANGLE_VERT_THRD{0.6};
};

struct FatigueStateConfig
{
    int EYE_CLOSE_TIME_THRD{3};
    int MOUTH_OPEN_TIME_THRD{2};
    double EYE_DGREE_THRD{0.32};
    double MOUTH_DGREE_THRD{0.35};
};

struct ModuleConfigs
{
    FaceRecognitionConfig FaceRecognitionCfg;
    FaceOcclusionConfig FaceOcclusionCfg;
    AbnormalBehaviorConfig AbnormalBehaviorCfg;
    FatigueStateConfig FatigueStateCfg;
};

// 纯静态类，独此一家，不完备的单例模式
class ConfigXmlParams
{
public:
    const static std::string cfgFilename;
    static ModuleConfigs Configs;

public:
    static void Init();

    // 操作变量的接口
    static bool Set_RECOGNITION_TIME_PERIOD(int time);

    static bool Set_EYES_OCCLUSION_TIME_THRD(int time);

    static bool Set_MOUTH_OCCLUSION_TIME_THRD(int time);

    static bool Set_FACE_ANGLE_HORIZ_TIME_THRD(int time);

    static bool Set_FACE_ANGLE_VIRT_TIME_THRD(int time);

    static bool Set_ANGLE_HORIZ_THRD(double angle);

    static bool Set_ANGLE_VERT_THRD(double angle);

    static bool Set_EYE_CLOSE_TIME_THRD(int time);

    static bool Set_MOUTH_OPEN_TIME_THRD(int time);

    static bool Set_EYE_DGREE_THRD(double dgree);

    static bool Set_MOUTH_DGREE_THRD(double dgree);

private:
    static bool readModuleConfigs();

    static bool saveModuleConfigs();

    static void parseModule(xmlNodePtr &pModule);// 根据功能模块解析参数

    static void parseFaceRecognition(xmlNodePtr &pFaceRecogModule);// 解析人脸识别

    static void parseFaceOcclusion(xmlNodePtr &pFaceOcclModule);// 解析人脸遮挡

    static void parseAbnormalBehavior(xmlNodePtr &pAtteDeviaModule);// 解析异常行为

    static void parseFatigueState(xmlNodePtr &pFatigueModule);// 解析疲劳状态

    static int convertXmlCharToInt(xmlChar *xmlChars);

    static double convertXmlCharToDouble(xmlChar *xmlChars);
};

#endif //DRIVER_MONITOR_CFG_PARAMS_H
