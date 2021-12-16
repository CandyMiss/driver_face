#include "face_gesture.h"
#include<iostream>
#include<math.h>

#include "../utils/cfg_params.h"

using std::cout;
using std::endl;

namespace PFLD
{
    const int NOSE_TIP_INDEX = 54;  //nose tip
    const int CHIN_INDEX = 16;  //chin
    const int L_EYE_L_CORNER_INDEX = 60;  //left eye left corner
    const int R_EYE_R_CORNER_INDEX = 72;  //right eye right corner
    const int L_MOUTH_INDEX = 88;  //left mouth corner
    const int R_MOUTH_INDEX = 92;  //right mouth corner
    const int NOSE_TOP_INDEX = 51;  //nose top
    const int MOUTH_TOP_INDEX = 79;  //mouth top
    const int L_EYE_START_INDEX = 60;  //left eye start
    const int R_EYE_START_INDEX = 68;  //right eye start

    float EAR_THRES = ConfigXmlParams::Configs.FatigueStateCfg.EYE_DGREE_THRD;  // ear算法 threshold
    float MAR_THRES = ConfigXmlParams::Configs.FatigueStateCfg.MOUTH_DGREE_THRD; //mar算法 threshold
    float FACE_LR_THRES = ConfigXmlParams::Configs.AbnormalBehaviorCfg.ANGLE_HORIZ_THRD;  //以1为正中心，左右移动的门限
    float UD_THRES_MAX = 1.0;  //updown threshold
    float UD_THRES_MIN = ConfigXmlParams::Configs.AbnormalBehaviorCfg.ANGLE_VERT_THRD;


    //距离公式
    float Distance(float x1, float y1, float x2, float y2)
    {
        return pow(pow(x1 - x2, 2) + pow(y1 - y2, 2), 0.5);
    }

    float PointDistance(struct Point a, struct Point b)
    {
        float x1 = a.x;
        float y1 = a.y;
        float x2 = b.x;
        float y2 = b.y;
        return pow(pow(x1 - x2, 2) + pow(y1 - y2, 2), 0.5);
    }

    //判断面部是否检测正确
    bool FaceValidation(float *prob)
    {
        //人脸
        struct Point nosetip = {prob[NOSE_TIP_INDEX * 2], prob[NOSE_TIP_INDEX * 2 + 1]};//鼻尖
        struct Point chin = {prob[CHIN_INDEX * 2], prob[CHIN_INDEX * 2 + 1]};//下巴
        struct Point leyelcorner = {prob[L_EYE_L_CORNER_INDEX * 2], prob[L_EYE_L_CORNER_INDEX * 2 + 1]};//左眼的左角
        struct Point reyercorner = {prob[R_EYE_R_CORNER_INDEX * 2], prob[R_EYE_R_CORNER_INDEX * 2 + 1]};//右眼的右角
//        struct Point nosetop = {prob[NOSE_TOP_INDEX * 2], prob[NOSE_TOP_INDEX * 2 + 1]};//鼻子最上边-眉心
        struct Point lmouth = {prob[L_MOUTH_INDEX * 2], prob[L_MOUTH_INDEX * 2 + 1]};//嘴巴左侧
        struct Point rmouth = {prob[R_MOUTH_INDEX * 2], prob[R_MOUTH_INDEX * 2 + 1]};//嘴巴右侧
//        struct Point mouthtop = {prob[MOUTH_TOP_INDEX * 2], prob[MOUTH_TOP_INDEX * 2 + 1]};//上嘴唇中心-嘴巴上部

        //人脸特征
        float nosetip_eye2nosetip_eye =
                PointDistance(nosetip, leyelcorner) / PointDistance(nosetip, reyercorner);//鼻尖到两眼角距离比
//        float nosetip_chin2nosetip_nosetop =
//                PointDistance(nosetip, chin) / PointDistance(nosetip, nosetop);//鼻尖到下颌与到眉心距离比

        //------------------------------ 备用判决数据------------------------------
        float nosetip_mouth2nosetip_mouth = PointDistance(nosetip, lmouth) / PointDistance(nosetip, rmouth);//鼻尖到两嘴角距离比
        float chin_eye2chin_eye = PointDistance(chin, leyelcorner) / PointDistance(chin, reyercorner);//下颌到两眼角距离比
        float chin_mouth2chin_mouth = PointDistance(chin, lmouth) / PointDistance(chin, rmouth);//下颌到两嘴角距离比
        float eye_dis2mouth_dis = PointDistance(leyelcorner, reyercorner) / PointDistance(lmouth, rmouth);//眼角距离比嘴角距离
//        float nosetip_mouthtop2nosetip_nosetop =
//                PointDistance(nosetip, mouthtop) / PointDistance(nosetip, nosetop);//鼻尖到嘴巴上部与到眉心距离比

        //只用了val1和val5做验证，若后期不够用再加上val2和val4
        if (nosetip_eye2nosetip_eye >= 0.2 && nosetip_eye2nosetip_eye <= 1.5
            && nosetip_mouth2nosetip_mouth >= 0.5 && nosetip_mouth2nosetip_mouth <= 2.5
            && chin_eye2chin_eye >= 0.85 && chin_eye2chin_eye <= 1.1
            && chin_mouth2chin_mouth >= 0.85 && chin_mouth2chin_mouth <= 1.2
            && eye_dis2mouth_dis >= 1.4 && eye_dis2mouth_dis <= 2.5)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    // EAR计算眼睛开合度
    float EyeEARResult(float *prob)
    {
        float lefteye[16];
        float righteye[16];
        int j = 0;
        for (int i = L_EYE_START_INDEX * 2; i <= L_EYE_START_INDEX * 2 + 15; i += 1)
        {  //lefteye in prob range 120-135
            lefteye[j] = prob[i];
            j++;
        }
        j = 0;
        for (int i = R_EYE_START_INDEX * 2; i <= R_EYE_START_INDEX * 2 + 15; i += 1)
        {  //righteye in prob range 136-151
            righteye[j] = prob[i];
            j++;
        }

        float leftEAR = (
                                Distance(lefteye[2], lefteye[3], lefteye[14], lefteye[15]) +
                                Distance(lefteye[4], lefteye[5], lefteye[12], lefteye[13]) +
                                Distance(lefteye[6], lefteye[7], lefteye[10], lefteye[11])
                        ) / (3 * Distance(lefteye[0], lefteye[1], lefteye[8], lefteye[9]));

        float rightEAR = (
                                 Distance(righteye[2], righteye[3], righteye[14], righteye[15]) +
                                 Distance(righteye[4], righteye[5], righteye[12], righteye[13]) +
                                 Distance(righteye[6], righteye[7], righteye[10], righteye[11])
                         ) / (3 * Distance(righteye[0], righteye[1], righteye[8], righteye[9]));

        return (leftEAR + rightEAR) / 2.f;
    }

    EyeState GetEyeState(bool isFaceValid, float ear)
    {
        if (!isFaceValid)
        {
//            cout << "眼睛没找到！" << endl;
            return EyeState::NONE;
        }

        if (ear <= EAR_THRES)
        {
//            cout << "眼睛闭" << endl;
            return EyeState::CLOSE;
        }
        else
        {
//            cout << "眼睛睁开" << endl;
            return EyeState::OPEN;
        }
    }

    //MAR方式判断嘴巴
    float MouthMARResult(float *prob)
    {
        float mouth[16];
        int j = 0;
        for (int i = L_MOUTH_INDEX * 2; i <= L_MOUTH_INDEX * 2 + 15; i += 1)
        {
            mouth[j] = prob[i];
            j++;
        }

        return (
                       Distance(mouth[2], mouth[3], mouth[14], mouth[15]) +
                       Distance(mouth[4], mouth[5], mouth[12], mouth[13]) +
                       Distance(mouth[6], mouth[7], mouth[10], mouth[11])
               ) / (3 * Distance(mouth[0], mouth[1], mouth[8], mouth[9]));
    }

    MouthState GetMouthState(bool isFaceValid, float mar)
    {
        if (!isFaceValid)
        {
            return MouthState::NONE;
        }

        if (mar >= MAR_THRES)
        {
            return MouthState::OPEN;
        }
        else
        {
            return MouthState::CLOSE;
        }
    }

    //左右方向姿态
    float FaceHorizAngle(float *prob)
    {
        struct Point nosetip = {prob[NOSE_TIP_INDEX * 2], prob[NOSE_TIP_INDEX * 2 + 1]};
        struct Point leyelcorner = {prob[L_EYE_L_CORNER_INDEX * 2], prob[L_EYE_L_CORNER_INDEX * 2 + 1]};
        struct Point reyercorner = {prob[R_EYE_R_CORNER_INDEX * 2], prob[R_EYE_R_CORNER_INDEX * 2 + 1]};

        float nosetip_eye2nosetip_eye = PointDistance(nosetip, leyelcorner) / PointDistance(nosetip, reyercorner);

        return nosetip_eye2nosetip_eye;
    }

    FaceDirectHoriz GetFaceDirectHoriz(bool isFaceValid, float angleHoriz)
    {
        if (!isFaceValid)
        {
            return FaceDirectHoriz::NONE;
        }

        float LR_THRES_MAX = 1 + FACE_LR_THRES;
        float LR_THRES_MIN = 1 - FACE_LR_THRES;
        if (angleHoriz <= LR_THRES_MAX && angleHoriz >= LR_THRES_MIN)
        {
            return FaceDirectHoriz::CENTER;
        }
        else if (angleHoriz < LR_THRES_MIN)
        {
            return FaceDirectHoriz::LEFT;
        }
        else
        {
            return FaceDirectHoriz::RIGHT;
        }
    }

    float FaceVirtAngle(float *prob)
    {
        struct Point nosetip = {prob[NOSE_TIP_INDEX * 2], prob[NOSE_TIP_INDEX * 2 + 1]};
        struct Point nosetop = {prob[NOSE_TOP_INDEX * 2], prob[NOSE_TOP_INDEX * 2 + 1]};
        struct Point mouthtop = {prob[MOUTH_TOP_INDEX * 2], prob[MOUTH_TOP_INDEX * 2 + 1]};


        float nosetip_mouthtop2nosetip_nosetop =
                PointDistance(nosetip, mouthtop) / PointDistance(nosetip, nosetop);//鼻尖到嘴巴上部与到眉心距离比

        return nosetip_mouthtop2nosetip_nosetop;
    }

    FaceDirectVert GetFaceDirectVert(bool isFaceValid, float angleVert)
    {
        if (!isFaceValid)
        {
            return FaceDirectVert::NONE;
        }

        if (angleVert <= UD_THRES_MAX && angleVert >= UD_THRES_MIN)
        {
            return FaceDirectVert::CENTER;
        }
        else if (angleVert < UD_THRES_MIN)
        {
            return FaceDirectVert::DOWN;
        }
        else
        {
            return FaceDirectVert::UP;
        }
    }
}
