#ifndef DRIVER_MONITOR_DATASET_H
#define DRIVER_MONITOR_DATASET_H

#include <iostream>

enum ClassID
{
    HEAD = 0,
    FACE,
    FACE_LEFT,
    FACE_RIGHT,
    FACE_UP,
    FACE_DOWN,
    CIGARETTE,
    PHONE,
    HAND_FIST,
    HAND_THUMB,
    HAND_INDEX_AND_MIDDLE_FINGER,
    HAND_BIG_AND_SMALL_THUMB,
    EYES_OCCLUSION,
    MOUTH_OCCLUSION
};

enum HandGesture
{
    NONE = 0,
    FIST,
    THUMB,
    BIG_AND_SMALL_THUMB,
    INDEX_AND_MIDDLE_FINGER
};

std::string GetNameFromID(float class_id);

#endif //DRIVER_MONITOR_DATASET_H
