#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/ResultMsg.h>
#include <driver_face/DriverIdMsg.h>
#include "font/CvxText.h"

#include <queue>
#include <map>
using std::queue;
using std::pair;
using cv::Mat;
using std::cout;
using std::endl;

const int TEXT_POSITION_X = 5;
const int TEXT_POSITION_Y = 40;
const int TEXT_POSITION_Y_STEP = 40;
const double FONT_SCALE = 2.5;


ros::Time StartStamp;
bool GotResult = false;
driver_face::ResultMsg reslut_msg;
std::string driveridstr = "不是司机！";

#pragma region 渲染中文字体
CvxText text("/home/nvidia/ros_vision/src/driver_face/src/res/MSYaHei.ttf"); //指定字体 ///home/nvidia/ros_vision/devel/res/MSYaHei.ttf    ./MSYaHei.ttf
cv::Scalar size1{40, 0.5, 0.1, 0}; // { 字体大小/空白比例/间隔比例/旋转角度 }
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
    char *str = (char *)"";
    switch(reslut_msg.FaceGesture)
    {
        case driver_face::ResultMsg::NONE:
            str = (char *)"NONE";
            break;
        case driver_face::ResultMsg::Face:
            // str = (char *)"Face";
            // strcpy(str, driveridstr.c_str());
            str = const_cast<char*> (driveridstr.c_str());
            break;
        default:
            break;
    }

    std::string testStr(str);
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    int baseline;
    cv::Size text_size = cv::getTextSize(testStr, font_face, FONT_SCALE, 2, &baseline);

    //将文本框居中绘制 
    int posX = canvas.cols / 2 - text_size.width / 2; 
    int posY = canvas.rows / 2 + text_size.height / 2;
    drawChineseChars(canvas, str, posX, posY, cv::Scalar(0, 0, 255)); //左上角绘制文字信息

    if(reslut_msg.FaceGesture != driver_face::ResultMsg::NONE)
    {
        cv::Rect r(reslut_msg.RectFace_x, reslut_msg.RectFace_y, reslut_msg.RectFace_w, reslut_msg.RectFace_h);
        cv::rectangle(canvas, r, cv::Scalar(0x27, 0xC1, 0x36), 2);  //在图像上绘制矩形
    }
}
#pragma endregion

ros::Time LatestResultStamp;
ros::Time CurAnalyzeStamp;
queue<pair<ros::Time, Mat>> FrameQueueFace;

//目标检测结果的回调函数，取出result_msg的结果存到全局量
void ResultMsgInfoCallback(const driver_face::ResultMsg::ConstPtr& msg)
{
    cout << "单帧分析耗时： " << (int)((msg->CurAnalyzeStamp - msg->LatestResultStamp).toSec() * 1000) << endl;
    GotResult = true;

    LatestResultStamp = msg->LatestResultStamp;
    CurAnalyzeStamp = msg->CurAnalyzeStamp;
    
    reslut_msg.FaceGesture = msg->FaceGesture;
    reslut_msg.RectFace_x = msg->RectFace_x;
    reslut_msg.RectFace_y = msg->RectFace_y;
    reslut_msg.RectFace_w = msg->RectFace_w;
    reslut_msg.RectFace_h = msg->RectFace_h;
}


//摄像头的消息回调函数
void DrawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        ros::Time stamp = msg->header.stamp;
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;   //整图

        if(GotResult)
        {
            // 1.删除废帧
            while(FrameQueueFace.size() > 0)
            {
                ros::Time tmp_stamp = FrameQueueFace.front().first;
                if(tmp_stamp < LatestResultStamp)
                {
                    FrameQueueFace.pop();
                }
                else
                {
                    break;
                }
            }
            cout << "剩余帧数：" << FrameQueueFace.size() << endl;
            for(unsigned int i = 0; i < FrameQueueFace.size(); i++)
            {
                cout << "I ";
            }
            cout << endl;

            // 2.符合条件的帧进入队列，队列长度控制在20以内
            if(FrameQueueFace.size() < 20)
            {
                FrameQueueFace.push(pair<ros::Time, Mat>(stamp, frame));
            }

            // 3.最老的一帧取出来渲染
            if(FrameQueueFace.size() > 0 && FrameQueueFace.front().first < CurAnalyzeStamp)
            {
                Mat canvas = FrameQueueFace.front().second.clone();
                FrameQueueFace.pop();
                drawRunning(canvas);    //在取出的一帧图像上绘制矩形
                cv::imshow("view", canvas);
            }
        }
        else
        {
            drawIniting(frame);
            cv::imshow("view", frame);
        } 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

//DriverIdMsg Callback
void imageCallback(const driver_face::DriverIdMsg::ConstPtr& msg)    
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
    ros::init(argc, argv, "image_render_node");
    ros::NodeHandle node_render;

    // 初始化字体
    int fontType;
    cv::Scalar fontSize;
    bool fontUnderline;
    float fontDiaphaneity;
    text.getFont(&fontType, &fontSize, &fontUnderline, &fontDiaphaneity);
    text.setFont(&fontType, &size1, &fontUnderline, &fontDiaphaneity);

    StartStamp = ros::Time::now();
    cv::namedWindow("view");
    // cv::namedWindow("view",  cv::WINDOW_NORMAL);
    // cv::setWindowProperty("view", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);   全屏显示
    cv::startWindowThread();

    ros::Subscriber person_info_sub = node_render.subscribe("/camera_csi0/cur_result", 1, ResultMsgInfoCallback);
    image_transport::ImageTransport it(node_render);
    image_transport::Subscriber sub = it.subscribe("/camera_csi0/frames", 1, DrawImageCallback);
    ros::Subscriber driver_id_sub = node_render.subscribe("/camera_csi0/driver_id", 1, imageCallback);    

    ros::spin();

    cv::destroyWindow("view");    //窗口

    return 0;
}