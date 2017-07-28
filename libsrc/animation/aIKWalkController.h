#ifndef IKWalkController_H_
#define IKWalkController_H_

#include "aIKController.h"
#include "aBVHController.h"
class IKWalkController
{
public:
    IKWalkController();
    virtual ~IKWalkController();

    void loadReferenceMotion(const std::string& filename);
    const Actor& getSkeleton() const;

    double getFramerate() const { return mBVHController.getFramerate(); }
    int getNumKeys() const { return mBVHController.getNumKeys(); }

    void edit(double strideScale, double heightScale);
    virtual void update(double dt);
    virtual void drawOpenGL(float time); // for debugging

protected:
    typedef std::vector<int> FootContacts;
    std::vector<vec3> getSupportPolygon(Actor& pose, int keyID, const vec3& up);
    void computeContacts(double velThreshold, double heightThreshold);

protected:

    BVHController mBVHController;
    IKController mLeftLegController;
    IKController mRightLegController;
    Actor mSkeleton;

    double mStepMin;

    AnimatableTransform* mLFoot;
    AnimatableTransform* mRFoot;
    vec3 startLPos, startRPos;

    // foot contact information
    std::vector<FootContacts> mContacts;

    // cached curves for debugging
    ASplineVec3 mOrigRootCurve; // left foot end effector curve
    ASplineVec3 mOrigLCurve; // left foot end effector curve
    ASplineVec3 mOrigRCurve; // right foot end effector curve

    ASplineVec3 mRootCurve; // left foot end effector curve
    ASplineVec3 mLCurve; // left foot end effector curve
    ASplineVec3 mRCurve; // right foot end effector curve

    std::vector<std::vector<vec3>> mLContacts;
    std::vector<std::vector<vec3>> mRContacts;
};

#endif