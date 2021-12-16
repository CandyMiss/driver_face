#ifndef DRIVER_MONITOR_DRAW_MASK_H
#define DRIVER_MONITOR_DRAW_MASK_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const float MASK_CENTER_X_RATE = 0.5;    // 遮罩位置中心的x坐标
static const float MASK_HEIGHT_RATE = 0.8;      // 遮罩的高度
static const float MASK_W_H_RATE = 0.75;        // 遮罩宽高比为4:3。为的是不变形

namespace ArcFace
{
    bool InitFaceIDMaterial(int canvasWidth, int canvasHeight);
    void DrawCanvas(cv::Mat &canvas, bool isFaceCapture);
    cv::Rect GetFacePos(int imgWidth, int imgHeight);
}

#endif //DRIVER_MONITOR_DRAW_MASK_H

//这个文件用来画人脸识别的框，框出需要检测的人脸的部分