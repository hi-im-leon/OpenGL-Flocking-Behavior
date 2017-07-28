#include "aParticleSystem.h"
#include "GL/glut.h"
#include <cstring>
#include <math.h>
#include "aVector.h"

#define GRAVITY 9.8f

AParticleSystem::AParticleSystem() :
    mParticles(),
    mRoot(0),
    mInfinite(true),
    mMaxParticles(10),
    mLifetime(5.0),
    mGravity(0, -9.8, 0),
    mStartColor(1, 0, 0), mEndColor(0, 0, 1),
    mStartPos(0, 0, 0),
    mStartVel(0, 9.8 * 2, 0),
    mStartAlpha(1.0), mEndAlpha(0.0),
    mStartScale(1), mEndScale(5),
    mScaleJitter(0, 1),
    mColorJitter(-0.25, 0.25),
    mPositionJitter(-5, 5),
    mVelocityJitter(-0.5, 0.5)
{
    for (int i = 0; i < mMaxParticles; i++)
    {
        mParticles.push_back(AParticle());
    }
}

AParticleSystem::~AParticleSystem()
{
}

bool AParticleSystem::isAlive()
{
    if (mInfinite) return true; // we never die!

    // TODO
    // Return true if inifinite
    // Else Return true if any particle is still alive
    return false;
}

void AParticleSystem::update(double dt)
{
    //vec3 force = mGravity;
    // TODO
    // for each particle
    //  if infinite AND a particle is not alive, initialize the particle to reset it
    //  else update the particle
}

void AParticleSystem::drawOpenGL()
{
    for (int i = 0; i < (int) mParticles.size(); i++)
    {
        if (!mParticles[i].isAlive()) continue;

        vec3 pos = mParticles[i].mPos;
        double scale = mParticles[i].mScale;
        vec3 color = mParticles[i].mColor;
        double alpha = mParticles[i].mAlpha;

        glColor4f(color[0], color[1], color[2], alpha);
        glPushMatrix();
        glTranslatef(pos[0], pos[1], pos[2]);
        glScalef(scale, scale, scale);
        glutSolidSphere(1.0, 10, 10);
        glPopMatrix();
    }
}

void AParticleSystem::reset()
{
    mParticles.clear();

    AJitter jitterTime(-mLifetime, 0);
    for (int i = 0; i < mMaxParticles; i++)
    {
        mParticles.push_back(AParticle());
        mParticles[i].init(*this);

        // for the starting particles only, stagger the time to live in infinite mode
        if (mInfinite) mParticles[i].mTtl += AJitterVal(jitterTime);
    }

}


AParticle::AParticle() :
    mTtl(10), mLifetime(10), 
    mMass(1), 
    mPos(0, 0, 0), 
    mVel(0, 0, 0),
    mColor(1, 1, 1), 
    mStartColor(1, 1, 1),
    mEndColor(1, 1, 1), 
    mScale(1), mStartScale(1), mEndScale(1),
    mAlpha(1), mStartAlpha(1), mEndAlpha(0)
{
}

AParticle::~AParticle()
{
}

void AParticle::init(const AParticleSystem& parent)
{
    mStartAlpha = parent.mStartAlpha;
    mEndAlpha = parent.mEndAlpha;
    mAlpha = mStartAlpha;

    mStartColor = parent.mStartColor + AJitterVec(parent.mColorJitter);
    mEndColor = parent.mEndColor + AJitterVec(parent.mColorJitter);
    mColor = mStartColor;

    mStartScale = parent.mStartScale + AJitterVal(parent.mScaleJitter);
    mEndScale = parent.mEndScale + AJitterVal(parent.mScaleJitter);
    mScale = mStartScale;

    mLifetime = parent.mLifetime;
    mTtl = mLifetime;
    mPos = parent.mStartPos + AJitterVec(parent.mPositionJitter);
    mVel = parent.mStartVel + AJitterVec(parent.mVelocityJitter);
}

bool AParticle::isAlive() const
{
    return mTtl >= 0;
}

void AParticle::update(double dt, const vec3& externalForces)
{
    if (mTtl < 0) return;

    // TODO
    // Update velocity and position based on external forces and mass
    mVel = vec3(0, 0, 0);
    mPos = vec3(0, 0, 0);

    // Update time to live based on dt
    mTtl = mTtl - dt;

    // Update scale, alpha, and color based on TTL and Lifetime using linear interpolation
    mScale = mStartScale;
    mAlpha = mStartAlpha;
    mColor = mStartColor;
}

