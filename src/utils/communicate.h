#ifndef DRIVER_MONITOR_COMMUNICATE_H
#define DRIVER_MONITOR_COMMUNICATE_H

#include <zmq.h>
#include <string>

// 通信格式：
// 00000010:RECOGNITION_TIME_PERIOD:6

static const unsigned int ZMQ_SIGNALING_LEN = 8;
static const std::string SIGNAL_HANDSHAKE = "00000000";
static const std::string SIGNAL_SET_PARAMS = "00000001";
static const std::string SIGNAL_TRAIN_SIGNAL = "00000010";
static const std::string SIGNAL_APP_STOP = "10000000";

class ZmqCommunicate
{
public:
    void *skt;

private:
    void *context;
    int rc;

public:
    ZmqCommunicate();

    ~ZmqCommunicate();

    void DealMessages(char *buffer);

private:
    void setParamsValue(char *buffer);
    void dealTrainSignal(char *buffer);
    void handShake(char *buffer);
};


#endif //DRIVER_MONITOR_COMMUNICATE_H
