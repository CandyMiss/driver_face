#include "draw_mask.h"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

namespace ArcFace
{
    Mat trans, head_contour;

    bool InitFaceIDMaterial(int canvasWidth, int canvasHeight)
    {
        trans = imread("trans.jpg");
        if (trans.cols == 0 || trans.rows == 0)
        {
            cout << "人脸遮罩素材丢失！" << endl;
            return false;
        }

        head_contour = imread("head_contour.jpg");

        if (head_contour.cols == 0 || head_contour.rows == 0)
        {
            cout << "人脸遮罩素材丢失！" << endl;
            return false;
        }

        resize(trans, trans, cv::Size(canvasHeight * MASK_HEIGHT_RATE * MASK_W_H_RATE, canvasHeight * MASK_HEIGHT_RATE));
        resize(head_contour, head_contour, cv::Size(canvasHeight * MASK_HEIGHT_RATE * MASK_W_H_RATE, canvasHeight * MASK_HEIGHT_RATE));
        return true;
    }

    struct Line
    {
        Point Start;
        Point End;
    };

    vector<Line> capFaceBox(Rect rect)
    {
        float lineLen = 0.3;
        vector<Line> lines;

        Point left_top(rect.x, rect.y);
        Point left_bottom(rect.x, rect.y + rect.height);
        Point right_top(rect.x + rect.width, rect.y);
        Point right_bottom(rect.x + rect.width, rect.y + rect.height);

        Line line1 = {left_top, Point(left_top.x + rect.width * lineLen, left_top.y)};
        Line line2 = {Point(right_top.x - rect.width * lineLen, right_top.y), right_top};
        Line line3 = {left_bottom, Point(left_bottom.x + rect.width * lineLen, left_bottom.y)};
        Line line4 = {Point(right_bottom.x - rect.width * lineLen, right_bottom.y), right_bottom};

        Line line5 = {left_top, Point(left_top.x, left_top.y + rect.height * lineLen)};
        Line line6 = {Point(left_bottom.x, left_bottom.y - rect.height * lineLen), left_bottom};
        Line line7 = {right_top, Point(right_top.x, right_top.y + rect.height * lineLen)};
        Line line8 = {Point(right_bottom.x, right_bottom.y - rect.height * lineLen), right_bottom};

        lines.push_back(line1);
        lines.push_back(line2);
        lines.push_back(line3);
        lines.push_back(line4);
        lines.push_back(line5);
        lines.push_back(line6);
        lines.push_back(line7);
        lines.push_back(line8);

        return lines;
    }

    cv::Rect getMaskPos(int imgWidth, int imgHeight)
    {
        int height = imgHeight * MASK_HEIGHT_RATE;
        int y = (imgHeight - height) / 2;

        int width = height * MASK_W_H_RATE;
        int x = imgWidth * MASK_CENTER_X_RATE - width / 2;

        assert(x >= 0 && y >= 0 && x + width <= imgWidth && y + height <= imgHeight && "遮罩位置超出画面范围");

        return cv::Rect(x, y, width, height);
    }

    void DrawCanvas(Mat &canvas, bool isFaceCapture)
    {
        Mat mask(canvas.rows, canvas.cols, CV_8UC3, Scalar(50, 50, 50));
        Rect rectRoi = getMaskPos(canvas.cols, canvas.rows);
        Mat roi = mask(rectRoi);

        bitwise_and(roi, trans, roi);
        roi = roi + head_contour;

        canvas = canvas + mask;

        if (isFaceCapture)
        {
            roi = canvas(rectRoi);
            int side = roi.cols;
            Rect rect(0, roi.rows - side, side, side);
            // 绘制8条小线段
            vector<Line> lines = capFaceBox(rect);
            for (vector<Line>::iterator iter = lines.begin(); iter < lines.end(); ++iter)
            {
                line(roi, iter->Start, iter->End, Scalar(0, 0, 255), 8);
            }
        }

        flip(canvas, canvas, 1);   //为了方便截图，临时翻转让司机对准面部
    }

    cv::Rect GetFacePos(int imgWidth, int imgHeight)
    {
        cv::Rect maskRect = getMaskPos(imgWidth, imgHeight);

        int y = maskRect.y + (maskRect.height - maskRect.width);

        return cv::Rect(maskRect.x, y, maskRect.width, maskRect.width);
    }
}
