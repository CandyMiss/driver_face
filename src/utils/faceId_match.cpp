#include "faceId_match.h"

void FaceIdMatchMaker::addFaceId(int id)
{
    if (faceIdRec.find(id) != faceIdRec.end()){
        faceIdRec[id] += 1;
    }
    else
    {
        faceIdRec[id] = 1;
    }
}

bool FaceIdMatchMaker::detectFaceId(int &id)
{
    detectTimes++;
    if (detectTimes >= 10)
    {
        detectTimes = 0;
        id = getMaxFaceId();
        faceIdRec.clear();
        return true;
    }

    return false;
}

int FaceIdMatchMaker::getMaxFaceId()
{
    int id = -1;
    int max = -1;

    for (auto pair: faceIdRec)
    {
        if(pair.second > max)
        {
            max = pair.second;
            id = pair.first;
        }
    }

    return id;
}

void FaceIdMatchMaker::reset()
{
    faceIdRec.clear();
    detectTimes = 0;
}

int FaceIdMatchMaker::getDetectTimes()
{
    return detectTimes;
}