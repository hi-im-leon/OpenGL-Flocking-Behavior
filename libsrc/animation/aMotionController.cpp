#include "aMotionController.h"
#include <GL/glut.h>
#pragma warning(disable:4018)

AMotionController::AMotionController() : mFps(120.0), mBlendTime(0.5), mNext(""), mTime(0)
{
}

AMotionController::~AMotionController()
{
    clear();
}

void AMotionController::clear()
{
    mSkeleton.clear();
    mMotions.clear();
}

void AMotionController::update(double time)
{
    if (mMotions.size() == 0) return;

    mTime = time;
    if (mCurrent == "Blend")
    {
        if ((mTime-mStartTime) < mBlend.duration())
        {
            setPose(mBlend, mTime);
        }
        else
        {
            mCurrent = mNext;
            mTime = mBlendTime; // ASN TODO: be more precise with time to avoid discontinuities
        }
    }

    if (mCurrent != "Blend") // re-check because state may have changed
    {
        setPose(mMotions[mCurrent], mTime);
    }
}

void AMotionController::setPose(AMotion& motion, double time)
{
    for (int i = 0; i < mSkeleton.getNumTransforms(); i++)
    {
        AnimatableTransform* joint = mSkeleton.getByID(i);
        if (i == 0)
        {
            vec3 pos = motion.mRootMotion.getValue(time);
            joint->setLocalTranslation(pos);
        }
        Quaternion q = motion.mMotion[i].getValue(time);
        joint->setLocalRotation(q.ToRotation());
    }
    mSkeleton.updateTransforms();
}

void AMotionController::drawOpenGL()
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

void AMotionController::setSkeleton(const Actor& actor)
{
    mSkeleton = actor;
}

void AMotionController::addMotion(const std::string& name, const AMotion& motion)
{
    mMotions[name] = motion;
    mCurrent = name;
}

void AMotionController::play(const std::string& name)
{
    if (mMotions.find(name) != mMotions.end())
    {        
        mNext = name;
        computeBlend(name, mBlendTime);
    }
}

void AMotionController::computeBlend(const std::string& nextMotion, double blendTime)
{
    AMotion& current = mMotions[mCurrent];
    AMotion& next = mMotions[mNext];
    mBlend.clear();

    // blend for blendTime seconds between current place in the motion and the beginning of the next
    double startTime = mTime;
    double endTime = mTime + blendTime;
    double dt = 1.0 / mFps;
    for (double time = startTime; time < endTime; time += dt)
    {
        double fraction = (time - startTime) / (endTime - startTime);

        vec3 currentPos = current.mRootMotion.getValue(time);
        vec3 nextPos = current.mRootMotion.getValue(time);
        vec3 blendPos = Lerp<vec3>(currentPos, nextPos, fraction);
        mBlend.mRootMotion.appendKey(blendPos);

        for (int j = 0; j < current.mMotion.size(); j++)
        {
            Quaternion currentQ = current.mMotion[j].getValue(time);
            Quaternion nextQ = next.mMotion[j].getValue(time);
            Quaternion blendQ = Lerp<Quaternion>(currentQ, nextQ, fraction);
            mBlend.mMotion[j].appendKey(blendQ);
        }
    }

    mCurrent = "Blend";
    mStartTime = mTime;
}

void AMotionController::stop()
{
}
