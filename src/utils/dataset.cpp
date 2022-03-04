#include "dataset.h"

std::string GetNameFromID(float class_id)
{
    int id = (int) class_id;
    switch (id)
    {
        case 0:
            return "CIGARETTE";
        case 1:
            return "PHONE";
        case 2:
            return "MOUTH_OCCLUSION";
        case 3:
            return "HAND_FIST";
        case 4:
            return "HAND_THUMB";
        case 5:
            return "HAND_BIG_AND_SMALL_THUMB";
        case 6:
            return "HAND_INDEX_AND_MIDDLE_FINGER";
        case 7:
            return "FACE";
        case 8:
            return "HEAD";
        case 9:
            return "EYES_OCCLUSION";
        case 10:
            return "fire hydrant";
        case 11:
            return "stop sign";
        case 12:
            return "parking meter";
        case 13:
            return "bench";
        case 14:
            return "bird";
        case 15:
            return "cat";
        case 16:
            return "dog";
        case 17:
            return "horse";
        case 18:
            return "sheep";
        case 19:
            return "cow";
        case 20:
            return "elephant";
        case 21:
            return "bear";
        case 22:
            return "zebra";
        case 23:
            return "giraffe";
        case 24:
            return "backpack";
        case 25:
            return "umbrella";
        case 26:
            return "handbag";
        case 27:
            return "tie";
        case 28:
            return "suitcase";
        case 29:
            return "frisbee";
        case 30:
            return "skis";
        case 31:
            return "snowboard";
        case 32:
            return "sports ball";
        case 33:
            return "kite";
        case 34:
            return "baseball bat";
        case 35:
            return "baseball glove";
        case 36:
            return "skateboard";
        case 37:
            return "surfboard";
        case 38:
            return "tennis racket";
        case 39:
            return "bottle";
        case 40:
            return "wine glass";
        case 41:
            return "cup";
        case 42:
            return "fork";
        case 43:
            return "knife";
        case 44:
            return "spoon";
        case 45:
            return "bowl";
        case 46:
            return "banana";
        case 47:
            return "apple";
        case 48:
            return "sandwich";
        case 49:
            return "orange";
        case 50:
            return "broccoli";
        case 51:
            return "carrot";
        case 52:
            return "hot dog";
        case 53:
            return "pizza";
        case 54:
            return "donut";
        case 55:
            return "cake";
        case 56:
            return "chair";
        case 57:
            return "sofa";
        case 58:
            return "pottedplant";
        case 59:
            return "bed";
        case 60:
            return "diningtable";
        case 61:
            return "toilet";
        case 62:
            return "tvmonitor";
        case 63:
            return "laptop";
        case 64:
            return "mouse";
        case 65:
            return "remote";
        case 66:
            return "keyboard";
        case 67:
            return "cell phone";
        case 68:
            return "microwave";
        case 69:
            return "oven";
        case 70:
            return "toaster";
        case 71:
            return "sink";
        case 72:
            return "refrigerator";
        case 73:
            return "book";
        case 74:
            return "clock";
        case 75:
            return "vase";
        case 76:
            return "scissors";
        case 77:
            return "teddy bear";
        case 78:
            return "hair drier";
        case 79:
            return "toothbrush";
    }

    return "";
}

