#include "aBVHController.h"
#include "aVector.h"
#include "aRotation.h"
#include <iostream>
#include <GL/glut.h>

using namespace std;

BVHController::BVHController() : mFps(120.0), mDt(0.008333)
{

}

BVHController::~BVHController()
{
    empty();
}

void BVHController::empty()
{
    mFilename = "";
    mSkeleton.clear();
    mRootMotion.clear();
    mMotion.clear();
}

void BVHController::update(double time)
{
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        AnimatableTransform* joint = mSkeleton.getByID(i);
        if (i == 0)
        {
            vec3 pos = mRootMotion.getValue(time);
            joint->setLocalTranslation(pos);
        }
        Quaternion q = mMotion[i].getValue(time);
        joint->setLocalRotation(q.ToRotation());
    }

    mSkeleton.updateTransforms();
}

Actor BVHController::getPose(double t) 
{
    update(t);
    return mSkeleton;
}

Actor BVHController::getKey(int key)
{
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        AnimatableTransform* joint = mSkeleton.getByID(i);
        if (i == 0)
        {
            vec3 pos = mRootMotion.getKey(key);
            joint->setLocalTranslation(pos);
        }
        Quaternion q = mMotion[i].getKey(key);
        joint->setLocalRotation(q.ToRotation());
    }

    mSkeleton.updateTransforms();
    return mSkeleton;
}


void BVHController::appendPose(double t, const Actor& pose)
{
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        AnimatableTransform* joint = pose.getByID(i);
        if (i == 0)
        {
            vec3 pos = joint->getLocalTranslation();
            mRootMotion.appendKey(t, pos);
        }
        Quaternion q = joint->getLocalRotation().ToQuaternion();
        mMotion[i].appendKey(t, q);        
    }
}

void BVHController::appendKey(int key, const Actor& pose)
{
    double time = key * (1.0 / getFramerate());
    appendPose(time, pose);
}


void BVHController::clear()
{
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        if (i == 0)
        {
            mRootMotion.clear();
        }
        mMotion[i].clear();
    }
}

void BVHController::drawOpenGL()
{
    glColor4f(0, 0, 1, 1);
    glBegin(GL_LINES);
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        AnimatableTransform* joint = mSkeleton.getByID(i);
        Joint* pParent = joint->getParent();
        if (!pParent) continue;

        vec3 startPosition = pParent->getGlobalTranslation();
        vec3 endPosition = joint->getGlobalTranslation();

        glVertex3f(startPosition[0], startPosition[1], startPosition[2]);
        glVertex3f(endPosition[0], endPosition[1], endPosition[2]);
    }
    glEnd();
}

bool BVHController::load(const std::string& filename)
{
    std::ifstream inFile(filename.c_str());
    if (!inFile.is_open())
    {
        std::cout << "WARNING: Could not open " << filename.c_str() << std::endl;
        return false;
    }

    empty();
    bool status = loadSkeleton(inFile) && loadMotion(inFile);

    if (status)
    {
        mFilename = filename;
        //std::string motionName = filename;
        //std::string annName = motionName.replace(motionName.end() - 4, motionName.end(), ".ann");
        //m_motion.LoadANNFile(annName, m_skeleton);
    }
    inFile.close();
    return status;
}

bool BVHController::loadSkeleton(std::ifstream& inFile)
{
    vec3 offsets;
    std::string readString, jointname;
    int channelCount;

    inFile >> readString;
    if (readString != "HIERARCHY")
        return false;
    inFile >> readString;
    if (readString != "ROOT" && readString != "JOINT")
        return false;
    inFile.get(); //" "
    getline(inFile, jointname);// joint name
    AnimatableTransform* joint = new AnimatableTransform(jointname);
    mSkeleton.addTransform(joint, true);
    inFile >> readString; // "{"
    inFile >> readString; // "OFFSET"
    inFile >> offsets[0] >> offsets[1] >> offsets[2];
    joint->setLocalTranslation(offsets);
    inFile >> readString;
    if (readString != "CHANNELS")
        return false;
    inFile >> channelCount;
    joint->setNumChannels(channelCount);
    getline(inFile, readString);	// " Xposition Yposition Zposition Zrotation Xrotation Yrotation"
    joint->setRotationOrder(readString);
    inFile >> readString;
    while (readString != "}")
    {
        if (!loadJoint(inFile, joint, readString))
        {
            return false;
        }
        inFile >> readString;
    }
    if (readString != "}") return false;

    mSkeleton.updateTransforms();
    return true;
}

bool BVHController::loadJoint(std::ifstream &inFile, Joint *pParent, std::string prefix)
{
    std::string readString, jointname;
    vec3 offsets;
    int channelCount;
    if (prefix == "JOINT")
    {
        inFile.get(); //" "
        getline(inFile, jointname);// joint name
        Joint* joint = new Joint(jointname);
        mSkeleton.addTransform(joint, false);
        AnimatableTransform::Attach(pParent, joint);
        inFile >> readString; // "{"
        inFile >> readString; // "OFFSET"
        inFile >> offsets[0] >> offsets[1] >> offsets[2];
        joint->setLocalTranslation(offsets);
        inFile >> readString; // "CHANNELS"
        inFile >> channelCount;
        joint->setNumChannels(channelCount);

        getline(inFile, readString);// " Zrotation Xrotation Yrotation"
        joint->setRotationOrder(readString);

        inFile >> readString; // "Joint" or "}" or "End"
        while (readString != "}")
        {
            if (loadJoint(inFile, joint, readString) == false)
                return false;
            inFile >> readString; // "Joint" or "}" or "End"
        }
        return true;
    }
    else if (prefix == "End")
    {
        inFile.get(); //" "
        getline(inFile, jointname);// joint name
        if (jointname.find("Site") != std::string::npos)
        {
            jointname = pParent->getName() + "Site";
        }

        Joint* joint = new Joint(jointname);
        joint->setNumChannels(0);
        mSkeleton.addTransform(joint, false);
        AnimatableTransform::Attach(pParent, joint);
        inFile >> readString; // "{"
        inFile >> readString; // "OFFSET"
        inFile >> offsets[0] >> offsets[1] >> offsets[2];
        joint->setLocalTranslation(offsets);
        inFile >> readString; // "}"
        return true;
    }
    else return false;
}

bool BVHController::loadMotion(std::ifstream& inFile)
{
    std::string readString;
    int frameCount;
    inFile >> readString;
    if (readString != "MOTION")
        return false;
    inFile >> readString;
    if (readString != "Frames:")
        return false;
    inFile >> frameCount;
    inFile >> readString; // "Frame"
    getline(inFile, readString); // " Time: 0.033333"
    mDt = atof(&(readString.c_str()[6]));
    mFps = 1.0 / mDt;

    // Init rotation curves
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        ASplineQuaternion q;
        q.setFramerate(mFps);
        q.setInterpolationType(ASplineQuaternion::LINEAR);
        mMotion[i] = q;
    }
    mRootMotion.setFramerate(mFps);
    mRootMotion.setInterpolationType(ASplineVec3::LINEAR);

    // Read frames
    for (int i = 0; i < frameCount; i++)
    {
       loadFrame(inFile);
    }

    mRootMotion.computeControlPoints();
    mRootMotion.cacheCurve();
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        mMotion[i].cacheCurve();
    }
    return true;
}

void BVHController::loadFrame(std::ifstream& inFile)
{
    float tx, ty, tz, r1, r2, r3;
    double t = mDt * mRootMotion.getNumKeys();
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        tx = ty = tz = 0.0f;
        r1 = r2 = r3 = 0.0f;

        Joint* pJoint = mSkeleton.getByID(i);
        if (pJoint->getNumChannels() == 6)
        {
            inFile >> tx >> ty >> tz;
            inFile >> r1 >> r2 >> r3;
        }
        else if (pJoint->getNumChannels() == 3)
        {
            inFile >> r1 >> r2 >> r3;
        }
        else
        {
        }

        if (mSkeleton.getRoot() == pJoint)
        {
            mRootMotion.appendKey(t, vec3(tx, ty, tz), false);
        }

        Quaternion quat = ComputeBVHRot(r1, r2, r3, pJoint->getRotationOrder());
        mMotion[i].appendKey(t, quat, false);
    }
}

Quaternion BVHController::ComputeBVHRot(float r1, float r2, float r3, const std::string& rotOrder) // For BVH
{
    mat3 m;
    float ry, rx, rz;

    if (rotOrder == "xyz")
    {
        rx = r1; ry = r2; rz = r3;
        m.FromEulerAnglesXYZ(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "xzy")
    {
        rx = r1; rz = r2; ry = r3;
        m.FromEulerAnglesXZY(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yxz")
    {
        ry = r1; rx = r2; rz = r3;
        m.FromEulerAnglesYXZ(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yzx")
    {
        ry = r1; rz = r2; rx = r3;
        m.FromEulerAnglesYZX(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zxy")
    {
        rz = r1; rx = r2; ry = r3;
        m.FromEulerAnglesZXY(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zyx")
    {
        rz = r1; ry = r2; rx = r3;
        m.FromEulerAnglesZYX(vec3(rx, ry, rz) * Deg2Rad);
    }
    return m.ToQuaternion();
}
