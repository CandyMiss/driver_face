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

void imageCallback(const driver_face::FaceRecMsg::ConstPtr& msg)
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
        if(msg->IsFace == true)
            cout<<" IsFace" <<endl;
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
    ros::init(argc, argv, "image_identify_node");
    ros::NodeHandle node_identify;
    cv::namedWindow("view2",cv::WINDOW_NORMAL); 

    ros::Subscriber sub = node_identify.subscribe("/camera_csi0/face_result", 1, imageCallback);    
    ros::spin();    
    cv::destroyWindow("view2");    //窗口

    return 0;

}


