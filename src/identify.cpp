#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <driver_face/FaceRecMsg.h>
#include <driver_face/DriverIdMsg.h>
#include "font/CvxText.h"
#include <iostream>

#include <queue>
#include <map>

#include "./pfld/pfld.h"
#include "./arcface/arcface.h"
#include "data/results.h"
#include "database/SqliteOp.h"
#include "utils/faceId_match.h"

using std::cout;
using std::endl;
using std::map;

// 当前结果寄存处

DriverIDResult *CurDriverIDResult;
Results *AllResult;
DriverResult driverResult;
driver_face::DriverIdMsg driver_msg;
FaceIdMatchMaker matchMaker;

ros::Publisher DriverInfoPub;

static float prob[PFLD::OUTPUT_SIZE];
static unsigned int DoFaceIDTimes = 0;
static int faceId = 0;         //tmp人脸id

float tmpFaceFeature[ArcFace::FACE_FEATURE_DIMENSION]{0.0};//512维特征向量
//cv::Mat faceQueue;


inline void printVector(float *tmpFaceFeature){ //用于辅助测试特征是否被捕捉
    for(int i=0; i<ArcFace::FACE_FEATURE_DIMENSION; ++i){
        cout << tmpFaceFeature[i] << " ";
    }
    cout << endl;
}

void imageCallback(const driver_face::FaceRecMsg::ConstPtr& msg)
{
    //sensor_msgs::Image ROS中image传递的消息形式
    try
    {
        boost::shared_ptr<void const> tracked_object;    //共享指针,原来初始化了：boost::shared_ptr<void const> tracked_object(&(msg->FaceImage))
        cv::Mat faceMat = cv_bridge::toCvShare(msg->FaceImage, tracked_object,"bgr8")->image;
        
        cv::imshow("view2", faceMat);    
        cv::waitKey(3); 
        
        if(msg->hasFace == true){
            ROS_INFO("hasFace");
                       if(msg->isMultiface == true){
                            ROS_INFO(" isMultiface");                      
                       }
        }else{
            ROS_INFO("no face");
        }

        
        if(msg->hasFace == true && msg->isMultiface==false){
            // //get tmp face feature
            // memset(tmpFaceFeature, 0, sizeof(tmpFaceFeature));
            // //printVector(tmpFaceFeature);
            // ArcFace::GetFaceFeature(faceMat, tmpFaceFeature);
            // //printVector(tmpFaceFeature);

            
            //do inference get face id
            ArcFace::DetectFaceID(faceMat, faceId, tmpFaceFeature);
            cout << "Face-----------------------------------------ID: " << faceId << endl << endl;
            matchMaker.addFaceId(faceId);
            // //得到人脸关键点
            // PFLD::AnalyzeOneFace(faceMat, prob);   // 正式使用时，改为faceMat//使用前要初始化引擎
            // driverResult.DealFaceResult(prob);
            // cout << "prob" << (*prob)<< endl;
        }
        else{
            //没找到可识别的人脸目标，或者追踪失败
            // DoFaceIDTimes = 0;
            // faceId = 0;
            // driverResult.ResetPointState();
            // driver_msg.isDriver = false;
            // driver_msg.driverID = faceId;
            matchMaker.addFaceId(0);
        }

        bool getSuccess = matchMaker.detectFaceId(faceId);
        if(getSuccess)
        {
            driver_msg.isDriver = true;
            if (faceId == 0)
            {
                driver_msg.isDriver = false;
            }
            driver_msg.driverID = faceId;
            DriverInfoPub.publish(driver_msg);
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
     // 初始化所有人脸数据
    ArcFace::ReadFaceDataToGPU();
    cout << "初始化人脸数据完成" <<  std::endl;   
    cout << "face teatrue:  "<< endl;


    cv::namedWindow("view2",cv::WINDOW_NORMAL); 
    ros::Subscriber sub = node_identify.subscribe("/camera_csi0/face_result", 1, imageCallback);    
    DriverInfoPub = node_identify.advertise<driver_face::DriverIdMsg>("/camera_csi0/driver_id", 1);

    ros::spin();   

    cv::destroyWindow("view2");    //窗口
    //YoloV5::ReleaseYoloV5Engine();
    PFLD::ReleasePFLDEngine();
    ArcFace::ReleasePFLDEngine();
    return 0;

}


