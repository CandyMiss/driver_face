#include <map>

class FaceIdMatchMaker
{
private:
    std::map<int, int> faceIdRec{};
    int detectTimes{0};
    int getMaxFaceId();

public:
    FaceIdMatchMaker() = default;

    void addFaceId(int faceId);

    bool detectFaceId(int &faceId);

    void reset();

    int getDetectTimes();
};