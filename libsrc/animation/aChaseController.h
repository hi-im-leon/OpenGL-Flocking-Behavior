#ifndef BVHCONTROLLER_H_
#define BVHCONTROLLER_H_

#include <map>
#include <string>
#include <fstream>
#include "aAnimatables.h"
#include "aSplineVec3.h"
#include "aSplineQuaternion.h"

class BVHController
{
public:
    BVHController();
    virtual ~BVHController();
    virtual void update(double time);
    virtual bool load(const std::string& filename);

    Actor& getSkeleton() { return mSkeleton;  }
    const Actor& getSkeleton() const { return mSkeleton; }
    int getNumJoints() const { return mSkeleton.getNumTransforms();  }

    int getNumKeys() const { return mRootMotion.getNumKeys();  }
    int getFrame(double time) const { return (int) (time/mDt) % getNumKeys(); }
    double getDuration() const { return mRootMotion.getDuration(); }
    double getFramerate() const { return mRootMotion.getFramerate(); }

    Actor getPose(double t);
    Actor getKey(int keyId);
    void appendPose(double t, const Actor& pose);
    void appendKey(int keyId, const Actor& pose);

    ASplineVec3& getRootCurve() { return mRootMotion;  }
    ASplineQuaternion& getJointCurve(int jointid) { return mMotion[jointid]; }

    virtual void clear(); 
    virtual void drawOpenGL(); // for debugging

protected:
    virtual Quaternion ComputeBVHRot(float r1, float r2, float r3, const std::string& rotOrder);
    virtual bool loadSkeleton(std::ifstream &inFile);
    virtual bool loadJoint(std::ifstream &inFile, AnimatableTransform *pParent, std::string prefix);
    virtual bool loadMotion(std::ifstream &inFile);
    virtual void loadFrame(std::ifstream& inFile);
    virtual void empty();

protected:
    std::string mFilename;
    Actor mSkeleton;
    double mFps;
    double mDt;
    ASplineVec3 mRootMotion;
    std::map<int, ASplineQuaternion> mMotion;

};

#endif
