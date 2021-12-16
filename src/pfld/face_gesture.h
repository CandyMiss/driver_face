#ifndef DRIVER_MONITOR_FACE_GESTURE_H
#define DRIVER_MONITOR_FACE_GESTURE_H

namespace PFLD
{

    struct Point
    {
        float x;
        float y;
    };

    enum class FaceDirectHoriz
    {
        NONE = -1,
        CENTER = 0,  //safe
        LEFT = 1,
        RIGHT = 2
    };

    enum class FaceDirectVert
    {
        NONE = -1,
        CENTER = 0,  //safe
        DOWN = 1,
        UP = 2
    };

    enum class MouthState
    {
        NONE = -1,
        OPEN = 0,
        CLOSE = 1  //safe
    };

    enum class EyeState
    {
        NONE = -1,
        OPEN = 1,   //safe
        CLOSE = 0
    };

    // 只有面部检测正确，才能够计算面部姿态数据
    bool FaceValidation(float *prob);

    float EyeEARResult(float *prob);

    EyeState GetEyeState(bool isFaceValid, float ear);

    float MouthMARResult(float *prob);

    MouthState GetMouthState(bool isFaceValid, float mar);

    float FaceHorizAngle(float *prob);

    FaceDirectHoriz GetFaceDirectHoriz(bool isFaceValid, float angleHoriz);

    float FaceVirtAngle(float *prob);

    FaceDirectVert GetFaceDirectVert(bool isFaceValid, float angleVert);
}

#endif //DRIVER_MONITOR_FACE_GESTURE_H
