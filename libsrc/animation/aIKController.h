#ifndef IKController_H_
#define IKController_H_

#include "aAnimatables.h"
class IKController
{
public:
    IKController();
    virtual ~IKController();

    void setSkeleton(Actor& actor) { mSkeleton = &actor; }
    Actor& getSkeleton() const { return *mSkeleton; }

    bool solveIKAnalytic(int jointid, const vec3& goalPos);
    bool solveIKCCD(int jointid, const vec3& goalPos, int chainSize = -1);

    virtual void drawOpenGL(); // for debugging

protected:
    void setSelectedJoint(int selectedJoint, int chainSize);

    std::vector<AnimatableTransform*> mIKChain; // from selected joint to parent
    int mSelectedJoint;
    int mChainSize;

public:

    Actor* mSkeleton;
    static double gIKEpsilon;
    static int gIKMaxIters;
};

#endif