#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/FaceRecMsg.h>
#include "font/CvxText.h"
#include <iostream>

#include <queue>
#include <map>

#include "./pfld/pfld.h"
#include "./arcface/arcface.h"
#include "data/results.h"
#include "database/SqliteOp.h"

using std::cout;
using std::endl;

// 当前结果寄存处

DriverIDResult *CurDriverIDResult;
Results *AllResult;
DriverResult driverResult;
static float prob[PFLD::OUTPUT_SIZE];
float tmpFaceFeature[ArcFace::FACE_FEATURE_DIMENSION]{0.0};//512维特征向量

void imageCallback(const driver_face::FaceRecMsg::ConstPtr& msg)
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
        if(msg->IsFace == true){
            ROS_INFO("IsFace");
                       if(msg->IsMultiFace == true){
                            ROS_INFO(" IsMultiFace");                      
                       }
        }else{
            ROS_INFO("no face");
        }
 
        
        boost::shared_ptr<void const> tracked_object;    //共享指针,原来初始化了：boost::shared_ptr<void const> tracked_object(&(msg->FaceImage))
        cv::Mat faceMat = cv_bridge::toCvShare(msg->FaceImage, tracked_object,"bgr8")->image;
        
        cv::imshow("view2", faceMat);    
        cv::waitKey(3); 
        
        if(msg->IsFace == true && msg->IsMultiFace==false){
            //get tmp face feature
            ArcFace::GetFaceFeature(faceMat, tmpFaceFeature);
            cout << (*tmpFaceFeature) << endl;
            //do inference get face id


            //得到人脸关键点
            PFLD::AnalyzeOneFace(faceMat, prob);   // 正式使用时，改为faceMat//使用前要初始化引擎
            driverResult.DealFaceResult(prob);

            cout << 'prob' << (*prob)<< endl;
        }
        else{
            driverResult.ResetPointState();
        }
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

    //防止崩溃，所有模型引擎优先初始化
    cudaSetDevice(DEVICE);
    ArcFace::InitArcFaceEngine();
    cout << "ArcFace 引擎序列化完成"  << std::endl;
    PFLD::InitPFLDEngine();
    cout << "PFLD 引擎序列化完成" <<  std::endl;

    cv::namedWindow("view2",cv::WINDOW_NORMAL); 
    ros::Subscriber sub = node_identify.subscribe("/camera_csi0/face_result", 1, imageCallback);    
    ros::spin();   

    cv::destroyWindow("view2");    //窗口

    return 0;

}


