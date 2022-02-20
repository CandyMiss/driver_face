#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/FaceRecMsg.h>
#include "font/CvxText.h"
#include <iostream>

#include <queue>
#include <map>

using std::cout;
using std::endl;

void imageCallback(const driver_face::FaceRecMsg::ConstPtr &msg)  //常量类型 对定义消息的常量指针的引用（msg）
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
        if(msg->IsFace == true)
            cout<<" IsFace" <<endl;
            //ROS_INFO("是否检测到人脸：%d",msg->Isface)//? 输出日志的写法 不知道是否正确
        if(msg->IsMultiFace == true)
            cout<<" IsMultiFace" <<endl;    
        // cout  <<  " IsFace" << msg->IsFace << "IsMultiFace" << msg->IsMultiFace << endl;
        boost::shared_ptr<void const> tracked_object;    //共享指针,原来初始化了：boost::shared_ptr<void const> tracked_object(&(msg->FaceImage))
        cv::Mat canvas = cv_bridge::toCvShare(msg->FaceImage, tracked_object,"bgr8")->image;
        //cv::circle(canvas, cv::Point(520, 430), 200, cv::Scalar(255, 0, 0), 8);
        cv::imshow("view2", canvas);    
        cv::waitKey(3); 
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'bgr8'.");
    }
}

int main(int argc, char **argv)
{
    //setlocale(LC_ALL,"");       //输出中文加这个不会乱码
    ros::init(argc, argv, "image_identify_node");
    ros::NodeHandle node_identify;
    cv::namedWindow("view2",cv::WINDOW_NORMAL); 

    ros::Subscriber sub = node_identify.subscribe("/camera_csi0/face_result", 1, imageCallback); //订阅者的消息范型可以不显示指出，可以自动推导   
    ros::spin();    //回头，处理回调函数，通常订阅者是spin，发布者是spinonce
    cv::destroyWindow("view2");    //窗口

    return 0;

}


