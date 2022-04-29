#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/ResultMsg.h>
#include <driver_face/FaceRecMsg.h>

#include "yolov5/yolov5.h"
#include "data/results.h"
#include "database/SqliteOp.h"

using namespace std;
using namespace std::chrono::_V2;

const int CAMERA_ID_Face = 0;

ros::Time CurAnalyzeStamp;
ros::Publisher StampInfoPub;
ros::Publisher FaceInfoPub;

DriverResult CurFaceResult;

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
		result_msg.LatestResultStamp = CurAnalyzeStamp;
        CurAnalyzeStamp = stamp;
		result_msg.CurAnalyzeStamp  = stamp;

        CurFaceResult.FaceCaptured = false;
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
        cout << "hasFace：" <<  (int)face_msg.hasFace << "   isMultiface：" <<  (int)face_msg.isMultiface << endl;
        cout << "Yolo V5 检测结果：" << CurFaceResult.toString() << endl;        
        if(CurFaceResult.FaceCaptured != false)
        {
            cout << "位置：" << rect.x << ", " << rect.y << endl;
        }

        //在消息回调函数里面发布另一个话题的消息
        StampInfoPub.publish(result_msg);
        FaceInfoPub.publish(face_msg);

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
    FaceInfoPub = node_analyze.advertise<driver_face::FaceRecMsg>("/camera_csi0/face_result", 1);

    image_transport::ImageTransport it(node_analyze);
    image_transport::Subscriber sub = it.subscribe("/camera_csi0/frames", 1, imageCallback);

    ros::spin();

    YoloV5::ReleaseYoloV5Engine();
}
