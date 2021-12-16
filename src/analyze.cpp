#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/ResultMsg.h>

#include "yolov5/yolov5.h"
#include "data/results.h"

using namespace std;
using namespace std::chrono::_V2;

const int CAMERA_ID_Face = 0;

ros::Time CurAnalyzeStamp;
ros::Publisher StampInfoPub;

DriverResult CurFaceResult;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
        ros::Time nowStamp = ros::Time::now();
        ros::Time stamp = msg->header.stamp;
        cout << "接收的图像是 " << (int)((nowStamp - stamp).toSec() * 1000) << " 毫秒之前拍摄的" << endl;

        cv::Mat frameFace = cv_bridge::toCvShare(msg, "bgr8")->image;
        driver_face::ResultMsg result_msg;
		result_msg.LatestResultStamp = CurAnalyzeStamp;
        CurAnalyzeStamp = stamp;
		result_msg.CurAnalyzeStamp  = stamp;

        // 准备发布yolo检测结果
        if(CurFaceResult.FaceCaptured)
        {
            result_msg.FaceGesture = 1;
        }
        else
        {
            result_msg.FaceGesture = 0;
        }
        // result_msg.FaceGesture = CurFaceResult.FaceCaptured;
        cv::Rect rect = YoloV5::get_rect(frameFace, CurFaceResult.RectFace); // 坐标转换
        result_msg.RectFace_x = rect.x;
        result_msg.RectFace_y = rect.y;
        result_msg.RectFace_w = rect.width;
        result_msg.RectFace_h = rect.height;

        StampInfoPub.publish(result_msg);

        // 得到分析结果，发布特定数据
        vector<Yolo::Detection> result = YoloV5::AnalyzeOneShot(frameFace);
        CurFaceResult.DealYoloResult(result); // rect先不做转换。在canvas上绘图时再具体生成坐标

        cout << "Yolo V5 检测结果：" << CurFaceResult.toString() << endl;
        if(CurFaceResult.FaceCaptured != FALSE)
        {
            cout << "位置：" << rect.x << ", " << rect.y << endl;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_analyze_node");
    ros::NodeHandle node_analyze;

    //防止崩溃，所有模型引擎优先初始化
    cudaSetDevice(DEVICE);
    YoloV5::InitYoloV5Engine();
    cout << "YoloV5 引擎序列化完成" << endl;

    CurAnalyzeStamp = ros::Time::now();
    StampInfoPub = node_analyze.advertise<driver_face::ResultMsg>("/camera_csi0/cur_result", 1);

    image_transport::ImageTransport it(node_analyze);
    image_transport::Subscriber sub = it.subscribe("/camera_csi0/frames", 1, imageCallback);

    ros::spin();

    YoloV5::ReleaseYoloV5Engine();
}
