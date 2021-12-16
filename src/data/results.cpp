#include "results.h"
#include "../pfld/face_gesture.h"
#include "../font/CvxText.h"
#include "../database/SqliteOp.h"

using namespace std;
using cv::Mat;
using Yolo::Detection;

const int TEXT_POSITION_X = 5;
const int TEXT_POSITION_Y = 40;
const int TEXT_POSITION_Y_STEP = 40;
const double FONT_SCALE = 2.5;

const double FACE_POINT_RECT_EXPANSIVITY = 1.3;

#pragma region DriverResult

void DriverResult::DealYoloResult(vector<Detection> &driverResult)
{
    // 仅保留面积最大的那个对象——对于单个驾驶员而言，以下所有的对象都是唯一的
    float areaHead{0.0};
    float areaFace{0.0};
    float areaEyeHead{0.0};
    float areaMouthHead{0.0};
    float areaCigarette{0.0};
    float areaPhone{0.0};

    for (vector<Detection>::iterator iter = driverResult.begin(); iter != driverResult.end(); ++iter)
    {
        switch ((int) (iter->class_id))
        {
            case ClassID::HEAD:
                if (iter->bbox[2] * iter->bbox[3] < areaHead)
                {
                    continue;
                }
                HeadCaptured = true;
                cout << "抓到头了！" << endl;
                memcpy(RectHead, iter->bbox, Yolo::LOCATIONS * sizeof(float));
                break;

#pragma region 看脸
            case ClassID::FACE:
                // 标明捕捉到脸，后续可执行脸部操作
                if (iter->bbox[2] * iter->bbox[3] < areaFace)
                {
                    continue;
                }
                FaceCaptured = true;
                cout << "抓到脸了！" << endl;
                memcpy(RectFace, iter->bbox, Yolo::LOCATIONS * sizeof(float));
                memcpy(RectFacePoint, RectFace, Yolo::LOCATIONS * sizeof(float));
                RectFacePoint[2] = RectFacePoint[2] * FACE_POINT_RECT_EXPANSIVITY;
                RectFacePoint[3] = RectFacePoint[3] * FACE_POINT_RECT_EXPANSIVITY;
                break;

            default:
                FaceCaptured = false;
        }
    }
}

std::string DriverResult::toString()
{
    switch (FaceCaptured)
    {
        case TRUE:
            return "检测到人脸";
        default:
            return "没检测到";
    }
}


