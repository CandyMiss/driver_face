#include <ros/ros.h>    
#include <image_transport/image_transport.h>    
#include <cv_bridge/cv_bridge.h>    
#include <opencv2/opencv.hpp>
#include <driver_face/DriverIdMsg.h>
#include <iostream>
    
void imageCallback(const driver_face::DriverIdMsg::ConstPtr& msg)    
{    
    //sensor_msgs::Image ROS中image传递的消息形式
    try    
    {   
        // 这里只需要空指针，不初始化，不加括号
        boost::shared_ptr<void const> tracked_object;    //共享指针,原来初始化了：boost::shared_ptr<void const> tracked_object(&(msg->FaceImage))
        if(msg->isDriver==true){
            std::cout << "isDriver: "<< "YES"<< "   driverID: "<< msg->driverID << std::endl;            
        }
        else{
            std::cout << "isDriver: "<< "NO"<< std::endl;
        }

        //cv::imshow("view2", canvas);    
        cv::waitKey(3);    
    }    
    catch (cv_bridge::Exception& e)    
    {    
        ROS_ERROR("Could not convert from image to 'bgr8'.");    
    }    
}    
    
int main(int argc, char **argv)    
{   
    ros::init(argc, argv, "get_driverid_node");    
    ros::NodeHandle nh;    
    //cv::namedWindow("view2",cv::WINDOW_NORMAL);    
    // cv::startWindowThread();    
    //image_transport::ImageTransport it(nh);    
    ros::Subscriber sub = nh.subscribe("/camera_csi0/driver_id", 1, imageCallback);    
    ros::spin();    
    //cv::destroyWindow("view2");    //窗口

}    
