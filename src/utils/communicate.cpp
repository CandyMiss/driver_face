#include "communicate.h"

#include <assert.h>
#include <string.h>
#include <sstream>

#include "../VideoThread.h"
#include "cfg_params.h"

using std::string;
using std::istringstream;

extern SIGNAL signal_exe;

static string zqmPort = "5555";

ZmqCommunicate::ZmqCommunicate()
{
    context = zmq_ctx_new();
    skt = zmq_socket(context, ZMQ_REP);

    string addr = "tcp://*:" + zqmPort;
    rc = zmq_bind(skt, addr.c_str());
    assert (rc == 0);
}

ZmqCommunicate::~ZmqCommunicate()
{
    zmq_close(skt);
    zmq_ctx_destroy(context);
}

inline void sendMsgBack(void *skt, char *buffer, string msg)
{
    strcat(buffer, msg.c_str());
    zmq_send(skt, buffer, strlen(buffer) + 1, 0);
}

void ZmqCommunicate::DealMessages(char *buffer)
{
    char strCode[ZMQ_SIGNALING_LEN + 1];
    strcpy(strCode, buffer);
    strCode[ZMQ_SIGNALING_LEN] = '\0';  // 截断字符数组，只保留8位码，并以'\0'结束
    string zmq_signaling(strCode);

    if (zmq_signaling == SIGNAL_HANDSHAKE)
    {
        handShake(buffer);
    }

    if (zmq_signaling == SIGNAL_SET_PARAMS)
    {
        // 要求参数必须一个一个修改，随时看反馈。通信时不支持批量参数修改
        setParamsValue(buffer);
        return;
    }

    // 列车信号处理
    if (zmq_signaling == SIGNAL_TRAIN_SIGNAL)
    {
        dealTrainSignal(buffer);
        return;
    }

    // 停止本程序的信号
    if (zmq_signaling == SIGNAL_APP_STOP)
    {
        signal_exe = SIGNAL::STOP;
        string answer = " Command Recieved. The Appllication Quit.";
        sendMsgBack(skt, buffer, answer);
    }
}

void ZmqCommunicate::setParamsValue(char *buffer)
{
    // 解析信息码后面的字符串
    char *paramBuffer = buffer + ZMQ_SIGNALING_LEN + 1; //定位
    string parmaMessage(paramBuffer);
    int colonIndex = parmaMessage.find(':');
    string paramName = parmaMessage.substr(0, colonIndex);
    string paramValue = parmaMessage.substr(colonIndex + 1);
    istringstream str2digit(paramValue);

    if (paramName == "RECOGNITION_TIME_PERIOD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_RECOGNITION_TIME_PERIOD(time))
        {
            // 发送回去消息：这个参数我搞定它的设置了！
            string answer = " Command Recieved. The Param RECOGNITION_TIME_PERIOD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "EYES_OCCLUSION_TIME_THRD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_EYES_OCCLUSION_TIME_THRD(time))
        {
            string answer =
                    " Command Recieved. The Param EYES_OCCLUSION_TIME_THRD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "MOUTH_OCCLUSION_TIME_THRD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_MOUTH_OCCLUSION_TIME_THRD(time))
        {
            string answer =
                    " Command Recieved. The Param MOUTH_OCCLUSION_TIME_THRD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "FACE_ANGLE_HORIZ_TIME_THRD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_FACE_ANGLE_HORIZ_TIME_THRD(time))
        {
            string answer =
                    " Command Recieved. The Param FACE_ANGLE_HORIZ_TIME_THRD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "FACE_ANGLE_VIRT_TIME_THRD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_FACE_ANGLE_VIRT_TIME_THRD(time))
        {
            string answer =
                    " Command Recieved. The Param FACE_ANGLE_VIRT_TIME_THRD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "ANGLE_HORIZ_THRD")
    {
        double angle;
        str2digit >> angle;
        if (ConfigXmlParams::Set_ANGLE_HORIZ_THRD(angle))
        {
            string answer = " Command Recieved. The Param ANGLE_HORIZ_THRD has been set " + std::to_string(angle);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "ANGLE_VERT_THRD")
    {
        double angle;
        str2digit >> angle;
        if (ConfigXmlParams::Set_ANGLE_VERT_THRD(angle))
        {
            string answer = " Command Recieved. The Param ANGLE_VERT_THRD has been set " + std::to_string(angle);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "EYE_CLOSE_TIME_THRD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_EYE_CLOSE_TIME_THRD(time))
        {
            string answer = " Command Recieved. The Param EYE_CLOSE_TIME_THRD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "MOUTH_OPEN_TIME_THRD")
    {
        int time;
        str2digit >> time;
        if (ConfigXmlParams::Set_MOUTH_OPEN_TIME_THRD(time))
        {
            string answer = " Command Recieved. The Param MOUTH_OPEN_TIME_THRD has been set " + std::to_string(time);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "EYE_DGREE_THRD")
    {
        double dgree;
        str2digit >> dgree;
        if (ConfigXmlParams::Set_EYE_DGREE_THRD(dgree))
        {
            string answer = " Command Recieved. The Param EYE_DGREE_THRD has been set " + std::to_string(dgree);
            sendMsgBack(skt, buffer, answer);
        }
    }

    if (paramName == "MOUTH_DGREE_THRD")
    {
        double dgree;
        str2digit >> dgree;
        if (ConfigXmlParams::Set_MOUTH_DGREE_THRD(dgree))
        {
            string answer = " Command Recieved. The Param MOUTH_DGREE_THRD has been set " + std::to_string(dgree);
            sendMsgBack(skt, buffer, answer);
        }
    }
}

void ZmqCommunicate::dealTrainSignal(char *buffer)
{

}

void ZmqCommunicate::handShake(char *buffer)
{
    buffer[0]='\0'; // 清空消息
    string answer = "Server Is Running. Server Connect Successful";
    sendMsgBack(skt, buffer, answer);
}
