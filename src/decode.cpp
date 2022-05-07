#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;
//设置gstreamer管道参数tx2s
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

// // //hk
// std::string gstreamer_pipeline(std::string uri, int latency, int display_width, int display_height) 
// {
//     return "rtspsrc location=" + uri +
//             " latency=" + std::to_string(latency) +
//             " ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, " + 
//             "width=(int)" + std::to_string(display_width) + 
//             ", height=(int)" + std::to_string(display_height) + 
//             ", format=(string)BGRx ! videoconvert ! appsink";
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_cam_node");  //argc:remapping 参数的个数，argv参数列表，运行时节点名
    ros::NodeHandle n;

//tx2
    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 10 ;
    int flip_method =2 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
                           capture_height,
                           display_width,
                           display_height,
                           framerate,
                           flip_method);
    


    //1 捕获视频
    cv::VideoCapture capture;
    capture.open(pipeline, cv::CAP_GSTREAMER);
    if(!capture.isOpened())
    {
        ROS_ERROR_STREAM("Failed to open camera!");
        ros::shutdown();
    }

    //2 创建ROS中图像的发布者
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image = it.advertise("/camera_csi0/frames", 1);

    //cv_bridge功能包提供了ROS图像和OpenCV图像转换的接口，建立了一座桥梁
    cv_bridge::CvImagePtr frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    while(ros::ok())
    {
        capture >> frame->image; //流的转换

        if(frame->image.empty())
        {
            ROS_ERROR_STREAM("Failed to capture frame!");
            ros::shutdown();
        }
        //打成ROS数据包
        frame->header.stamp = ros::Time::now();
        cv::Mat img = frame->image;
        
        //倒置画面，可以，但是画面上的日期信息也会倒置，不方面查看
        cv::Mat pubimg;
        flip(img,pubimg,-1);
        frame->image = pubimg;
        //test
        // imshow("img:",img); //tx2原摄像头倒置的
	    // imshow("pubimg: -1",pubimg);    //正向
        
        pub_image.publish(frame->toImageMsg());

        cv::waitKey(3);//opencv刷新图像 3ms
        ros::spinOnce();
    }

    capture.release();    //释放流
    return EXIT_SUCCESS;
}
