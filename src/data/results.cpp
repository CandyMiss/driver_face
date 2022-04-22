#include "results.h"
#include "../pfld/face_gesture.h"
#include "../font/CvxText.h"
#include "../database/SqliteOp.h"
#include "../pfld/pfld.h"


using namespace std;
using cv::Mat;
using Yolo::Detection;

const int TEXT_POSITION_X = 5;
const int TEXT_POSITION_Y = 40;
const int TEXT_POSITION_Y_STEP = 40;
const double FONT_SCALE = 2.5;

const double FACE_POINT_RECT_EXPANSIVITY = 1.3;

#pragma region DriverResult

void DriverResult::DealYoloResult(vector<Detection> &driverResult)  //会统计是别到的人脸数量
{
    // 仅保留面积最大的那个对象——对于单个驾驶员而言，以下所有的对象都是唯一的
    float areaHead{0.0};
    float areaFace{0.0};
   //float areaEyeHead{0.0};
    //float areaMouthHead{0.0};
    //float areaCigarette{0.0};
    //float areaPhone{0.0};
    FaceNum = 0;

    for (vector<Detection>::iterator iter = driverResult.begin(); iter != driverResult.end(); ++iter)
    {
        FaceCaptured = false;
        switch ((int) (iter->class_id))
        {
            // case ClassID::HEAD:
            //     if (iter->bbox[2] * iter->bbox[3] < areaHead)
            //     {
            //         continue;
            //     }
            //     HeadCaptured = true;
            //     cout << "抓到头了！" << endl;
            //     memcpy(RectHead, iter->bbox, Yolo::LOCATIONS * sizeof(float));
            //     break;

#pragma region 看脸
            case ClassID::FACE:
            {
                // 标明捕捉到脸，后续可执行脸部操作
                FaceNum ++;
                if (iter->bbox[2] * iter->bbox[3] < areaFace)
                {
                    continue;
                }
                FaceCaptured = true;
                cout << "抓到脸了！" << endl;
                memcpy(RectFace, iter->bbox, Yolo::LOCATIONS * sizeof(float));
                memcpy(RectFacePoint, RectFace, Yolo::LOCATIONS * sizeof(float));
                RectFacePoint[2] = RectFacePoint[2] * FACE_POINT_RECT_EXPANSIVITY;
                RectFacePoint[3] = RectFacePoint[3] * FACE_POINT_RECT_EXPANSIVITY;
                break;
            }


            default:
            {
                FaceCaptured = false;
                break;
            }

        }
    }

#pragma endregion
}

std::string DriverResult::toString()
{
    switch (FaceCaptured)
    {
        case TRUE:
            return "检测到人脸";
        default:
            return "没检测到";
    }
}

#pragma endregion

#pragma region DriverIDResult

DriverIDResult::DriverIDResult()
{
    const std::string face_map = "facemap.txt";
    const std::string suffix = ".jpg";
    unsigned int suffix_len = suffix.length();

    ifstream infile;
    infile.open(face_map.data());   //将文件流对象与文件连接起来
    assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

    string line;
    while (getline(infile, line))
    {
        //不需要添加面罩框了
        // if (line.length() > suffix_len)
        // {
        //     DriverIds.push_back(line.substr(0, line.length() - suffix_len));
        // }
    }
    infile.close();
}

void DriverIDResult::Reset()
{
    ID = 0;
    memset(CurDriveFaceFeature, 0, ArcFace::FACE_FEATURE_DIMENSION * sizeof(float));
    faceIDRec.clear();
    faceFeatureRec.clear();
}

void DriverIDResult::AddOneRecogResult(int faceID, float *faceFeature)  //一次识别结果，找id是否在map和人脸是否在map
{
    if (faceIDRec.end() != faceIDRec.find(faceID))                      //找到
    {
        faceIDRec[faceID]++;                                            //计数+1
        memcpy(faceFeatureRec[faceID], faceFeature, ArcFace::FACE_FEATURE_DIMENSION * sizeof(float));   //取出人脸特征
    }
    else
    {
        faceIDRec.insert(pair<int, int>(faceID, 1));
        faceFeatureRec.insert(pair<int, float *>(faceID, faceFeature));
    }
}

void DriverIDResult::GetIDReuslt()                                      //找多次识别之后最匹配的人
{
    int numIDMax = 0;                                                                   //默认司机id从0开始
    for (map<int, int>::iterator iter = faceIDRec.begin(); iter != faceIDRec.end(); ++iter)
    {
        if (numIDMax < iter->second)
        {
            ID = iter->first;                                           //记录识别最多的id
            numIDMax = iter->second;
        }
    }

    memcpy(CurDriveFaceFeature, faceFeatureRec[ID], ArcFace::FACE_FEATURE_DIMENSION * sizeof(float));

    cout << "确定驾驶员身份：" << ID << endl;
}

// string DriverIDResult::QueryDriverName()
// {
//     string id = DriverIds[ID];
//     DriverDataOp op;
//     op.Open();
//     string name = op.QueryDriverName(id);
//     op.Close();
//     return name;
// }

#pragma endregion

void DriverResult::DealFaceResult(float *pointsFace)
{
    analyzeFaceState(pointsFace);
    getFacePointMapPos(pointsFace);
}


void DriverResult::ResetPointState()
{
    FaceAngleHoriz = -1.0;
    FaceAngelVirt = -1.0;
    EyeOpen = -1.0;
    MouthOpen = -1.0;
}

void DriverResult::analyzeFaceState(float *pointsFace)
{
//    暂且认为脸部捕捉都是准的
//    IsFaceValid = PFLD::FaceValidation(pointsFace);
    cout << "脸部有效记录(实时计算)：" << IsFaceValid << endl;

    if (FaceCaptured)   // 当前只分析正脸
    {
        FaceAngleHoriz = PFLD::FaceHorizAngle(pointsFace);
        FaceAngelVirt = PFLD::FaceVirtAngle(pointsFace);
        EyeOpen = PFLD::EyeEARResult(pointsFace);
        MouthOpen = PFLD::MouthMARResult(pointsFace);
    }
    else
    {
        FaceAngleHoriz = -1.0;
        FaceAngelVirt = -1.0;
        EyeOpen = -1.0;
        MouthOpen = -1.0;
    }
}

void DriverResult::getFacePointMapPos(float *pointsFace)
{
    // center_x center_y w h转换(目前还是608*608范围，这是YoloV5的遗留问题)
    float faceLeft = RectFacePoint[0] - RectFacePoint[2] / 2.f;
    float faceWidth = RectFacePoint[2];
    float faceTop = RectFacePoint[1] - RectFacePoint[3] / 2.f;
    float faceHeight = RectFacePoint[3];
    // 所有面部关键点映射成yolo的608*608范围
    for (unsigned int i = 0; i < PFLD::POINT_NUM; i++)
    {
        // x_yolo = x_face * w_yolo + left_yolo，y坐标类似
        PointsFace[i * 2] = pointsFace[i * 2] * faceWidth + faceLeft;
        PointsFace[i * 2 + 1] = pointsFace[i * 2 + 1] * faceHeight + faceTop;
    }
}