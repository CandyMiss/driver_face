#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/ResultMsg.h>
#include <driver_face/FaceRecMsg.h>
#include <driver_face/TargetRes.h>
#include <driver_face/DriverIdMsg.h>
#include "font/CvxText.h"

#include "yolov5/yolov5.h"
#include "data/results.h"
#include "database/SqliteOp.h"

#include <queue>
#include <map>

using namespace std;
using namespace std::chrono::_V2;
using std::queue;
using std::pair;
using cv::Mat;
using std::cout;
using std::endl;

const int CAMERA_ID_Face = 0;

ros::Time CurAnalyzeStamp;
ros::Publisher StampInfoPub;
ros::Publisher FaceInfoPub;
ros::Publisher TargetInfoPub;

DriverResult CurFaceResult;

ros::Time LatestResultStamp;
queue<pair<ros::Time, Mat>> FrameQueueFace;

//渲染
const int TEXT_POSITION_X = 5;
const int TEXT_POSITION_Y = 40;
const int TEXT_POSITION_Y_STEP = 40;
const double FONT_SCALE = 2.5;


bool GotResult = false;
std::string driveridstr = "不是司机！";

#pragma region 渲染中文字体
CvxText text("/home/nvidia/ros_vision/src/driver_face/src/res/MSYaHei.ttf"); //指定字体 ///home/nvidia/ros_vision/devel/res/MSYaHei.ttf    ./MSYaHei.ttf
cv::Scalar size1{40, 0.5, 0.1, 0}; // { 字体大小/空白比例/间隔比例/旋转角度 }

//渲染相关函数
static int ToWchar(char *&src, wchar_t *&dest, const char *locale = "zh_CN.utf8")
{
    if (src == NULL)
    {
        dest = NULL;
        return 0;
    }

    // 根据环境变量设置locale
    setlocale(LC_CTYPE, locale);

    // 得到转化为需要的宽字符大小
    int w_size = mbstowcs(NULL, src, 0) + 1;

    // w_size = 0 说明mbstowcs返回值为-1。即在运行过程中遇到了非法字符(很有可能使locale没有设置正确)
    if (w_size == 0)
    {
        dest = NULL;
        return -1;
    }

    //wcout << "w_size" << w_size << endl;
    dest = new wchar_t[w_size];
    if (!dest)
    {
        return -1;
    }

    int ret = mbstowcs(dest, src, strlen(src) + 1);
    if (ret <= 0)
    {
        return -1;
    }
    return 0;
}

void drawChineseChars(cv::Mat &frame, char *str, int pos_x, int pos_y, cv::Scalar color)
{
    wchar_t *w_str;
    ToWchar(str, w_str);
    text.putText(frame, w_str, cv::Point(pos_x, pos_y), color);
    // cv::putText(frame, str, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_COMPLEX, 2, color, 2, 8, 0);
}

void drawIniting(cv::Mat& canvas)
{
    char *str = (char *)"初始化...";
    std::string testStr(str);
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    int baseline;
    cv::Size text_size = cv::getTextSize(testStr, font_face, FONT_SCALE, 2, &baseline);

    //将文本框居中绘制 
    int posX = canvas.cols / 2 - text_size.width / 2; 
    int posY = canvas.rows / 2 + text_size.height / 2;
    drawChineseChars(canvas, str, posX, posY, cv::Scalar(0, 0, 255)); //左上角绘制文字信息
}

//框出人脸，显示分析结果，传整图
void drawRunning(cv::Mat& canvas)
{
    // // yolo的结果使用yolo的坐标变换
    //     cv::Rect rFace = YoloV5::get_rect(canvas, currectFace);
        // cv::Rect rHead = YoloV5::get_rect(canvas, rectHead);
        // cv::rectangle(canvas, rFace, cv::Scalar(255, 0, 0), 2);
        // cv::rectangle(canvas, rHead, cv::Scalar(255, 0, 0), 2);
    text.setFont(nullptr, &size1, nullptr, 0);
    // std::string testStr(str);
    char *str = (char *)"";
    // int font_face = cv::FONT_HERSHEY_COMPLEX;
    // int baseline;
    // cv::Size text_size = cv::getTextSize(testStr, font_face, FONT_SCALE, 2, &baseline);
    // //将文本框居中绘制,统一绘制文字
    // int posX = canvas.cols / 2 - text_size.width / 2; 
    // int posY = canvas.rows / 2 + text_size.height / 2;


    if(CurFaceResult.HeadCaptured || CurFaceResult.FaceCaptured || CurFaceResult.FaceLeftCaptured || CurFaceResult.FaceRightCaptured || CurFaceResult.FaceUpCaptured || CurFaceResult.FaceDownCaptured ||
        CurFaceResult.IsEyeOcclusion || CurFaceResult.IsMouthOcclusion )
    {
        if(CurFaceResult.FaceCaptured)
        {
            // yolo的结果使用yolo的坐标变换
            cv::Rect rFace = YoloV5::get_rect(canvas, CurFaceResult.RectFace);
            cv::rectangle(canvas, rFace, cv::Scalar(255, 0, 0), 2);
        
            str = const_cast<char*> (driveridstr.c_str());
            drawChineseChars(canvas, str, TEXT_POSITION_X + TEXT_POSITION_Y_STEP * 5, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }
        if (CurFaceResult.FaceLeftCaptured)
        {
            char *str = (char *) "面部左转！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }

        if (CurFaceResult.FaceRightCaptured)
        {
            char *str = (char *) "面部右转！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }

        if (CurFaceResult.FaceUpCaptured)
        {
            char *str = (char *) "面部上抬！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }

        if (CurFaceResult.FaceDownCaptured)
        {
            char *str = (char *) "面部低下！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }

        if (CurFaceResult.IsEyeOcclusion)
        {
            char *str = (char *) "眼部遮挡！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }

        if (CurFaceResult.IsMouthOcclusion)
        {
            char *str = (char *) "嘴部遮挡！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255));
        }


        // if (CurFaceResult.IsDozeNod)
        // {
        //     char *str = (char *) "瞌睡低头！";
        //     drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y + TEXT_POSITION_Y_STEP * 2, cv::Scalar(0, 0, 255));
        // }


        // 2.闭眼驾驶————逻辑需要修改
        if (CurFaceResult.IsEyeClosed)
        {
            // yolo视野
            cv::rectangle(canvas, YoloV5::get_rect(canvas, CurFaceResult.rectEyeLeft), cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::rectangle(canvas, YoloV5::get_rect(canvas, CurFaceResult.rectEyeRight), cv::Scalar(0x27, 0xC1, 0x36), 2);

            char *str = (char *) "闭眼驾驶！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y + TEXT_POSITION_Y_STEP * 3, cv::Scalar(0, 0, 255));
        }

        // 3.打哈欠————逻辑需要修改
        if (CurFaceResult.IsYawn)
        {
            // yolo视野
            cv::rectangle(canvas, YoloV5::get_rect(canvas, CurFaceResult.rectMouth), cv::Scalar(0x27, 0xC1, 0x36), 2);

            char *str = (char *) "打哈欠，疲劳驾驶！";
            drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y + TEXT_POSITION_Y_STEP * 4, cv::Scalar(0, 0, 255));
        }
    }
    else if(CurFaceResult.HasCigarette)//香烟
    {
         // yolo的结果使用yolo的坐标变换
        cv::Rect rect = YoloV5::get_rect(canvas, CurFaceResult.RectCigarette);
//        cv::rectangle(frame, rect, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::rectangle(canvas, rect, cv::Scalar(0, 255, 0), 2);

        char *str = (char *) "有抽烟行为！";
        drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y + TEXT_POSITION_Y_STEP * 5, cv::Scalar(0, 255, 0));
    }
    else if(CurFaceResult.HasPhone)//手机
    {
         // yolo的结果使用yolo的坐标变换
        cv::Rect rect = YoloV5::get_rect(canvas, CurFaceResult.RectPhone);
        cv::rectangle(canvas, rect, cv::Scalar(0x27, 0xC1, 0x36), 2);

        char *str = (char *) "行驶期间使用手机！";
        drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y + TEXT_POSITION_Y_STEP * 6, cv::Scalar(0, 0, 255));
    }
    else
    {
        str = (char * )"无";
        drawChineseChars(canvas, str, TEXT_POSITION_X, TEXT_POSITION_Y, cv::Scalar(0, 0, 255)); //左上角绘制文字信息
    }






}
#pragma endregion

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
        ros::Time nowStamp = ros::Time::now();
        ros::Time stamp = msg->header.stamp;
        cout << "接收的图像是 " << (int)((nowStamp - stamp).toSec() * 1000) << " 毫秒之前拍摄的" << endl;

        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;       //摄像头接收到的 图像
        driver_face::ResultMsg result_msg;
        driver_face::FaceRecMsg face_msg;
        driver_face::TargetRes target_msg; //other target topic
		result_msg.LatestResultStamp = CurAnalyzeStamp;
        CurAnalyzeStamp = stamp;
		result_msg.CurAnalyzeStamp  = stamp;

        //set results flase;
        CurFaceResult.FaceCaptured = false;
        CurFaceResult.HeadCaptured = false;
        // result_msg.FaceGesture = CurFaceResult.FaceCaptured;
        cv::Rect rect = YoloV5::get_rect(frame, CurFaceResult.RectFace); // 坐标转换，得到目标框在图中的位置
        // 得到分析结果
        vector<Yolo::Detection> result = YoloV5::AnalyzeOneShot(frame); //返回存储bundingbox的vector
        CurFaceResult.DealYoloResult(result);                           //分析目标检测的结果
        

        // 下标容易越界，必须保护一下，不然会异常崩溃
        if (rect.x > 0 && rect.y > 0 && rect.width > 0 && rect.height > 0 &&
            (rect.x + rect.width) < frame.cols &&
            (rect.y + rect.height) < frame.rows)
        {
            result_msg.RectFace_x = rect.x;
            result_msg.RectFace_y = rect.y;
            result_msg.RectFace_w = rect.width;
            result_msg.RectFace_h = rect.height;
        }
        else
        {
            CurFaceResult.FaceCaptured = false;
            result_msg.RectFace_x = 0;
            result_msg.RectFace_y = 0;
            result_msg.RectFace_w = 0;
            result_msg.RectFace_h = 0;
        }


        cv::Mat face_frame;

        //whether catch the head.
        face_msg.hasHead = CurFaceResult.HeadCaptured;
        // 准备发布yolo检测结果，是否抓到人脸
        if(CurFaceResult.FaceCaptured)
        {
            result_msg.FaceGesture = 1;
            face_msg.hasFace = true;
            face_frame = frame(rect); //截出人脸的图像
        }
        else
        {
            result_msg.FaceGesture = 0;
            face_msg.hasFace = false;
            face_frame = frame; //截出人脸的图像
        }
        //是否多张人脸
        if(CurFaceResult.FaceCaptured && CurFaceResult.FaceNum > 1){
            face_msg.isMultiface = true;
        }
        else{
            face_msg.isMultiface = false;
        }

        face_msg.FaceImage = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", face_frame ).toImageMsg());  //用cv_bridge转化mat
        
        //提示信息
        cout << "hasHead: " << (int)face_msg.hasHead << endl;
        cout << "hasFace：" <<  (int)face_msg.hasFace << "   isMultiface：" <<  (int)face_msg.isMultiface << endl;
        cout << "Yolo V5 检测结果：" << CurFaceResult.toString() << endl;        
        if(CurFaceResult.FaceCaptured != false)
        {
            cout << "位置：" << rect.x << ", " << rect.y << endl;
        }
        //init other target msg
        target_msg.isHeadNod = false;
        target_msg.isPhoneAround = false;
        target_msg.isGazeLeaveRail = false;
        target_msg.isEyeShield = false;
        target_msg.isMouthShield = false;
        target_msg.isFoundCig = false;
        //generate other target from yolo result
        for (auto it : result)
        {
            if ((int) (it.class_id) == 3) {
                target_msg.isGazeLeaveRail = true;
                target_msg.isHeadNod = true;
            }
            else if ((int) (it.class_id) == 7)
                target_msg.isPhoneAround = true;
            else if ((int) (it.class_id) == 1 || (int) (it.class_id) == 2 || (int) (it.class_id) == 3 || (int) (it.class_id) == 4)
                target_msg.isGazeLeaveRail = true;
            else if ((int) (it.class_id) == 5)
                target_msg.isEyeShield = true;
            else if ((int) (it.class_id) == 6)
                target_msg.isMouthShield = true;
            else if ((int) (it.class_id) == 8)
                target_msg.isFoundCig = true;
        }

        //渲染
        Mat canvas = frame;
        // cv::imshow("view_frame", canvas);
        drawRunning(canvas);    //在取出的一帧图像上绘制矩形
        cv::imshow("render_view", canvas);

        //在消息回调函数里面发布另一个话题的消息
        StampInfoPub.publish(result_msg);
        FaceInfoPub.publish(face_msg);
        TargetInfoPub.publish(target_msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



//DriverIdMsg Callback
void IDInfoCallback(const driver_face::DriverIdMsg::ConstPtr& msg)    
{    
    //sensor_msgs::Image ROS中image传递的消息形式
    try    
    {   
        // 这里只需要空指针，不初始化，不加括号
        boost::shared_ptr<void const> tracked_object;    //共享指针,原来初始化了：boost::shared_ptr<void const> tracked_object(&(msg->FaceImage))
        if(msg->isDriver==true){
            driveridstr = "司机id:" + std::to_string(msg->driverID);          
        }
        else{
            driveridstr = "不是司机！";
        }
        std::cout << driveridstr << std::endl;
        //cv::imshow("view2", canvas);    
        //cv::waitKey(3);    
    }    
    catch (cv_bridge::Exception& e)    
    {    
        ROS_ERROR("Could not convert from image to 'bgr8'.");    
    }    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_analyze_node");
    ros::NodeHandle node_analyze;

    // 初始化字体
    int fontType;
    cv::Scalar fontSize;
    bool fontUnderline;
    float fontDiaphaneity;
    text.getFont(&fontType, &fontSize, &fontUnderline, &fontDiaphaneity);
    text.setFont(&fontType, &size1, &fontUnderline, &fontDiaphaneity);

    //防止崩溃，所有模型引擎优先初始化
    cudaSetDevice(DEVICE);
    YoloV5::InitYoloV5Engine();
    cout << "YoloV5 引擎序列化完成" << endl;

    cv::namedWindow("render_view");
    cv::startWindowThread();

    CurAnalyzeStamp = ros::Time::now();
    StampInfoPub = node_analyze.advertise<driver_face::ResultMsg>("/camera_csi0/cur_result", 1);
    FaceInfoPub = node_analyze.advertise<driver_face::FaceRecMsg>("/camera_csi0/face_result", 1);
    TargetInfoPub = node_analyze.advertise<driver_face::TargetRes>("/camera_csi0/target_result", 1);

    image_transport::ImageTransport it(node_analyze);
    image_transport::Subscriber sub = it.subscribe("/camera_csi0/frames", 1, imageCallback);
    ros::Subscriber driver_id_sub = node_analyze.subscribe("/camera_csi0/driver_id", 1, IDInfoCallback);    

    ros::spin();

    YoloV5::ReleaseYoloV5Engine();
    cv::destroyWindow("render_view");    //窗口
}
