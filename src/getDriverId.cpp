#include <ros/ros.h>    
#include <image_transport/image_transport.h>    
#include <cv_bridge/cv_bridge.h>    
#include <opencv2/opencv.hpp>
#include <driver_face/DriverIdMsg.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <map>
#include <string>

using std::map;
    
const std::string suffix = ".jpg";
const std::string separator = "_";
std::string map_path = "//home/nvidia/ros_vision/src/driver_face/src/res/";

map<int, std::string> id_name_map;
void imageCallback(const driver_face::DriverIdMsg::ConstPtr& msg)    
{    
    //sensor_msgs::Image ROS中image传递的消息形式
    try    
    {   
        // 这里只需要空指针，不初始化，不加括号
        boost::shared_ptr<void const> tracked_object;    //共享指针,原来初始化了：boost::shared_ptr<void const> tracked_object(&(msg->FaceImage))
        if(msg->isDriver==true){
            std::cout << "isDriver: "<< "YES"<< "   driverID: "<< msg->driverID << "    Name: " << id_name_map[ msg->driverID] << std::endl;            
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
    
    //初始化映射表
    int id;
    std::string name;
    //读txt到map
    std::ifstream ins(map_path + "face_map.txt");
    while(!ins.eof())
    {
        ins >> id >> name;
        id_name_map.insert(make_pair(id, name));
    }
    for(map<int,std::string>::iterator iter = id_name_map.begin(); iter != id_name_map.end(); ++iter){
        std::cout<<"key:"<<iter->first<<" value:"<<iter->second<<std::endl;
    }
    std::cout << "初始化映射表完成。" << std::endl;
    ros::spin();    
    //cv::destroyWindow("view2");    //窗口

}    
