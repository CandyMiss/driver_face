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
    int FaceNum{0};

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
    int ID{0};
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


class HandResult
{
public:
    HandGesture Gesture;
    // 只有一只手！——如果有多只，那就找最大的！
    float RectHand[Yolo::LOCATIONS];    // Yolo::LOCATIONS 就是4。rect存储比值，center_x center_y w h

public:
    HandResult(bool isFist, bool isThumb, bool isBigAndSmallThumb, bool isIndexAndMiddleFinger);

    void DealYoloResult(std::vector<Yolo::Detection> handResult);

    std::string toString();

private:
    float getAreaOfRect(Yolo::Detection obj);

    void dealResultClass(Yolo::Detection &obj);
};


class Results
{
public:
    template<typename T>
    void PushOneRec(std::vector<std::pair<TimeStamp, T>> &records, std::pair<TimeStamp, T> &rec, unsigned int duration)
    {
        // 记录中必须有2个以上的记录
        if (records.size() < 2)
        {
            std::cout << "Result处理：少于2个元素，必须push" << std::endl;

            records.push_back(rec);
            return;
        }

        auto newStamp = rec.first.toMillisecond();
        auto secondOldestStamp = records[1].first.toMillisecond();

        // 保证首尾元素的时间差，恰恰好大于duration
        while (std::chrono::duration_cast<std::chrono::milliseconds>(newStamp - secondOldestStamp).count() >=
               duration * 1000)
        {
            if (records.size() < 2)
            {
                std::cout << "Result处理：Result处理：少于2个元素，不能再pop了" << std::endl;

                break;
            }

            records.erase(records.begin()); //删除头部元素：删除一个过于久远的元素
//            std::cout << "删除一个过于久远的元素" << std::endl;

            secondOldestStamp = records[1].first.toMillisecond();
        }

        records.push_back(rec);
//        std::cout << "Result处理：新元素插入" << std::endl;
    }

public:
    // 当前最新结果的时间戳：用于渲染图像时的对准依据
    TimeStamp CurDriverStamp;
    TimeStamp CurHandStamp;

#pragma region 具有持续时间特征的结果记录数据
    // 用于存储一段时间内的一手资料。遮挡是定性数据，转头、闭眼、张嘴是定量数据
    // 之所以使用vector不用queue，是因为queue无法随机访问元素，在队列需要根据元素间关系而调整个数时，queue特别难办。
    std::vector<std::pair<TimeStamp, bool>> HeadCaptureRecords;
    std::vector<std::pair<TimeStamp, bool>> FaceCaptureRecords;
    std::vector<std::pair<TimeStamp, bool>> FaceLeftCaptureRecords;
    std::vector<std::pair<TimeStamp, bool>> FaceRightCaptureRecords;
    std::vector<std::pair<TimeStamp, bool>> FaceUpCaptureRecords;
    std::vector<std::pair<TimeStamp, bool>> FaceDownCaptureRecords;
    std::vector<std::pair<TimeStamp, bool>> EyeOcclusionRecords;
    std::vector<std::pair<TimeStamp, bool>> MouthOcclusionRecords;
    std::vector<std::pair<TimeStamp, bool>> IsFaceValidRecords;
    std::vector<std::pair<TimeStamp, float>> FaceAngleHorizRecords;
    std::vector<std::pair<TimeStamp, float>> FaceAngelVirtRecords;
    std::vector<std::pair<TimeStamp, float>> EyeOpenRecords;
    std::vector<std::pair<TimeStamp, float>> MouthOpenRecords;
#pragma endregion

#pragma region 结果数据
    // 测试数据
    bool HeadCaptured{false};
    bool FaceCaptured{false};
    bool FaceLeftCaptured{false};
    bool FaceRightCaptured{false};
    bool FaceUpCaptured{false};
    bool FaceDownCaptured{false};

    // 一手数据，可直接作为判定结果：遮眼，遮嘴；抓住抽烟、电话；人脸数据；手势
    bool IsEyeOcclusion{false};
    bool IsMouthOcclusion{false};
    bool HasCigarette{false};
    bool HasPhone{false};

    HandGesture Gesture;

    // 需经过计算的结果：在连续时间内，计算并判断注意力分散、瞌睡、闭眼、哈欠
    bool IsDistracted{false};
    bool IsDozeNod{false};
    bool IsEyeClosed{false};
    bool IsYawn{false};
#pragma endregion

    float pointsFace[PFLD::OUTPUT_SIZE]{0.0};   // ------------这里测试先用Public，以后要改为private
private:

#pragma region 用于画面渲染的素材
//    float pointsFace[PFLD::OUTPUT_SIZE]{0.0};

    float rectHead[Yolo::LOCATIONS]{0.0};    // 人头。Yolo::LOCATIONS 就是4。rect存储比值，center_x center_y w h
    float rectFace[Yolo::LOCATIONS]{0.0};    // 人脸
    float rectFacePoint[Yolo::LOCATIONS]{0.0};    // 人脸关键点检测所用。它应当是rectFace之外更广阔一点的范围，需要包括双耳及额头

    float rectCigarette[Yolo::LOCATIONS]{0.0};
    float rectPhone[Yolo::LOCATIONS]{0.0};

    float rectEyeLeft[Yolo::LOCATIONS]{0.0};// 左眼[center_x center_y w h]
    float rectEyeRight[Yolo::LOCATIONS]{0.0};// 右眼[center_x center_y w h]
    float rectMouth[Yolo::LOCATIONS]{0.0};

    float rectHand[Yolo::LOCATIONS]{0.0};// 手势
#pragma endregion

public:
    Results() : Gesture(HandGesture::NONE)
    {

    }

    // stage1：存入结果
    void PushResult(TimeStamp stampDriver, DriverResult &driverResult, TimeStamp stampHand, HandResult &handResult);

    // stage2：分析结果逻辑
    void AnalyzeState();

    // stage3：绘制分析画面——两路
    void DrawDriverResult(cv::Mat &frame);

    void DrawHandResult(cv::Mat &frame);

    // stage4：给通信发送报警信号


    // 两路视频，分别存入分析结果————等两路视频确定后，这两个函数要改为private
    void pushDriverResult(TimeStamp stamp, DriverResult &driverResult);

    void pushHandResult(TimeStamp stamp, HandResult &handResult);

private:
#pragma region 将单次的数值结果存入记录——主要针对需要持续性判断的数据

    void pushHeadCapture(TimeStamp stamp, bool isHeadCaptured);

    void pushFaceCapture(TimeStamp stamp, bool isFaceCaptured);
    void pushFaceLeftCapture(TimeStamp stamp, bool isFaceLeftCaptured);
    void pushFaceRightCapture(TimeStamp stamp, bool isFaceRightCaptured);
    void pushFaceUpCapture(TimeStamp stamp, bool isFaceUpCaptured);
    void pushFaceDownCapture(TimeStamp stamp, bool isFaceDownCaptured);

    void pushEyeOcclusion(TimeStamp stamp, bool isEyeOcclusion);

    void pushMouthOcclusion(TimeStamp stamp, bool ismouthOcclusion);

    void pushIsFaceValid(TimeStamp stamp, bool isValid);

    void pushFaceAngleHoriz(TimeStamp stamp, float faceAngleHoriz);

    void pushFaceAngelVirt(TimeStamp stamp, float faceAngelVirt);

    void pushEyeOpen(TimeStamp stamp, float eyeOpen);

    void pushMouthOpen(TimeStamp stamp, float mouthOpen);

#pragma endregion

#pragma region 各种状态的分析逻辑

    bool getFaceValidValue(TimeStamp stamp);

    bool getFaceDownCaptured(TimeStamp stamp);

    void analyzeIsDistracted();

    void analyzeIsDozeNod();

    void analyzeIsEyeClosed();

    void analyzeIsYawn();

#pragma endregion

#pragma region 绘制界面的工具函数

    void getLeftEyeRect();

    void getRightEyeRect();

    void getMouthRect();

#pragma endregion

    std::string getHandGestureName();
};


