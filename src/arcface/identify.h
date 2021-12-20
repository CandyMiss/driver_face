#include "../arcface/draw_mask.h"
#include "../arcface/arcface.h"
#include "../gst/gst_decode.h"

//每次加一帧到yoloInputs尾端
bool yoloPushOneInput(vector<Mat> &yoloInputs, queue<pair<TimeStamp, Mat>> &FrameQueue, Mat &frame,qQ TimeStamp &stamp);

//识别人脸id
void doFaceID(Mat &frameDriver, TimeStamp &stampDriver, DriverResult &driverResult);
//识别司机面部id的主功能函数
void doDriverID(vector<Mat> &yoloInputs);

void TheSameDriverWhileRunning(Mat &frameDriver, TimeStamp &stampDriver, DriverResult &driverResult);

enum class SIGNAL
{
    INIT = 0,
    FACE_ID_RECOG,
    RUN,
    STOP
};
