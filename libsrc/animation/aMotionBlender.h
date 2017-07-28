#ifndef MotionBlender_h_
#define MotionBlender_h_

#include "aBVHController.h"

class MotionBlender
{
public:
    MotionBlender();
    virtual ~MotionBlender();

    virtual void loadMotion1(const std::string& m1);
    virtual void loadMotion2(const std::string& m2);
    virtual void blend(int startFrame, int endFrame, int numBlendFrames);
    virtual void update(double dt);

    Actor& getSkeleton() { return mBlend.getSkeleton(); }
    const Actor& getSkeleton() const { return mBlend.getSkeleton(); }

    int getNumKeys1() const { return mMotion1.getNumKeys(); }
    int getNumKeys2() const { return mMotion2.getNumKeys(); }
    int getNumKeys() const { return mBlend.getNumKeys(); }
    double getFramerate() const { return mMotion1.getFramerate();  }

protected:    
    void append(BVHController& input, int startKeyId, int endKeyId, BVHController& output);

    void align(BVHController& motion1, BVHController& motion2,
        int startKeyId, int endKeyId, int numBlendFrames);

    void crossfade(BVHController& motion1, BVHController& motion2,
        int startKeyId, int endKeyId, int numBlendFrames, BVHController& blend);

protected:

    BVHController mBlend;
    BVHController mMotion1;
    BVHController mMotion2;
};

#endif