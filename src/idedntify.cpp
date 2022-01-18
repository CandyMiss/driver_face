#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/FaceRecMsg.h>
#include "font/CvxText.h"

#include <queue>
#include <map>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_identify_node");
    ros::NodeHandle node_identify;

    image_transport::ImageTransport it(node_identify);
    image_transport::Subscriber sub = it.subscribe("/camera_csi0/face_result", 1, imageCallback);

    ros::spin();

}


void imageCallback(const driver_face::FaceRecMsg::ConstPtr& msg)
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
       cout  <<  " IsFace" << msg->IsFace << "IsMultiFace" << msg->IsMultiFace << endl;
       cout  <<  "image" << 
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
