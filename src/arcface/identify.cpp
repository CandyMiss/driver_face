#include "../arcface/draw_mask.h"
#include "../arcface/arcface.h"
#include "../gst/gst_decode.h"

DriverIDResult *CurDriverIDResult;
float tmpFaceFeature[ArcFace::FACE_FEATURE_DIMENSION]{0.0};

//识别人脸id
void doFaceID(Mat &frameDriver, TimeStamp &stampDriver, DriverResult &driverResult)
{
    cv::Rect faceRect = YoloV5::get_rect(frameDriver, driverResult.RectFace);

    // 下标容易越界，必须保护一下，不然会异常崩溃
    if (faceRect.x > 0 && faceRect.y > 0 && faceRect.width > 0 && faceRect.height > 0 &&
        (faceRect.x + faceRect.width) < frameDriver.cols &&
        (faceRect.y + faceRect.height) < frameDriver.rows)
    {
        //没找到这个函数，直接用facerect？
        Mat faceMat = frameDriver(faceRect);        //frameDriver没有声明和定义

        cout << "人脸处于正确位置，开始分析驾驶员身份..." << endl;
        
        FaceIDRecogTimeStamp = stampDriver;

        int faceId;
        //引用返回人脸id
        ArcFace::DetectFaceID(faceMat, faceId, tmpFaceFeature);
        CurDriverIDResult->AddOneRecogResult(faceId, tmpFaceFeature);

        cout << "人脸数据-----------------------------------------ID: " << faceId << endl << endl;
    }
}
//识别司机面部id的主功能函数
void doDriverID(vector<Mat> &yoloInputs)
{
    TimeStamp stampDriver;
    Mat frameDriver;
    bool isGoodInput = yoloPushOneInput(yoloInputs, FrameQueueDriver, frameDriver,
                                        stampDriver);   // 确定时间戳并给yolo喂Driver的数据
    if (!isGoodInput)
    {
        return;
    }

    CurDriverRendTimeStamp = CurDriverAnalyzeTimeStamp;
    CurDriverAnalyzeTimeStamp = stampDriver;

    vector<Yolo::Detection> result = YoloV5::AnalyzeOneShot(frameDriver);
    DriverResult driverResult;
    driverResult.DealYoloResult(result);

    AllResult->pushDriverResult(stampDriver, driverResult); // 为了绘图测试临时使用的代码，后期可删。人脸识别的结果数据不必记录到疲劳检测结果中

    if (driverResult.FaceCaptured)
    {
        cv::Rect faceRect = YoloV5::get_rect(frameDriver, driverResult.RectFace);
        cv::Rect maskFaceRect = ArcFace::GetFacePos(frameDriver.cols, frameDriver.rows);
        cout << "faceRect: " << faceRect.x << ", " << faceRect.y << ", " << faceRect.width << ", " << faceRect.height << endl;
        cout << "maskFaceRect: " << maskFaceRect.x << ", " << maskFaceRect.y << ", " << maskFaceRect.width << ", " << maskFaceRect.height << endl;
        cout << "IOU: " << ResultTool::IOU(faceRect, maskFaceRect) << endl;

        if (ResultTool::IOU(faceRect, maskFaceRect) >= IOU_FACE_RECT_THRD)
        {
            isCapFace = true;

            doFaceID(frameDriver, CurDriverAnalyzeTimeStamp, driverResult);

            // 建议人脸识别多做几次，把身份确定了，然后再跳入正常业务中
            DoFaceIDTimes++;
            if (DoFaceIDTimes > DriverIDResult::FACE_RECOG_TIMES)
            {
                CurDriverIDResult->GetIDReuslt();
                DoFaceIDTimes = 0;
                signal_exe = SIGNAL::RUN;
            }
        }
        else
        {
            isCapFace = false;
            return;
        }
    }
    else
    {
        isCapFace = false;
    }
}

//每次加一帧到yoloInputs尾端
bool yoloPushOneInput(vector<Mat> &yoloInputs, queue<pair<TimeStamp, Mat>> &FrameQueue, Mat &frame,qQ TimeStamp &stamp)
{
    // 获取Driver的帧数据
    MTX_LOCK_Driver.lock();
    pair<TimeStamp, Mat> time_frame = FrameQueueDriver.back();
    // 防止读到坏帧
    if (time_frame.second.rows == 0 || time_frame.second.cols == 0)
    {
        MTX_LOCK_Driver.unlock();
        return false;
    }
    stamp = time_frame.first;
    frame = time_frame.second.clone();
    MTX_LOCK_Driver.unlock();
    yoloInputs.push_back(frame);

    return true;
}

void TheSameDriverWhileRunning(Mat &frameDriver, TimeStamp &stampDriver, DriverResult &driverResult)
{
    cv::Rect faceRect = YoloV5::get_rect(frameDriver, driverResult.RectFace);

    if (faceRect.x > 0 && faceRect.y > 0 && faceRect.width > 0 && faceRect.height > 0 &&
        (faceRect.x + faceRect.width) < frameDriver.cols &&
        (faceRect.y + faceRect.height) < frameDriver.rows)
    {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                stampDriver.toMillisecond() - FaceIDRecogTimeStamp.toMillisecond()).count() >=
            FACE_ID_TIME_PERIOD)
        {
            // 执行人脸识别并返回身份验证信息
            Mat faceMat = frameDriver(faceRect);
            ArcFace::GetFaceFeature(faceMat, tmpFaceFeature);
            float similar = ArcFace::GetSimilarOfTwoFace(tmpFaceFeature, CurDriverIDResult->CurDriveFaceFeature);
            cout << "运行期间人脸相似度：" << similar << endl;
            if (similar < FACE_SIMILAR_RUNNING_THRD)
            {
                // TODO : 业务如何处理？？？？？？仅仅报警？还是要退出监测直到等待驾驶员到来
                signal_exe = SIGNAL::FACE_ID_RECOG;
            }
            FaceIDRecogTimeStamp = stampDriver;

        }
    }
}

