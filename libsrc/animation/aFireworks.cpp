#include "aFireworks.h"
#include <GL/glut.h>
#include <cmath>
//---------------------

ASpark::ASpark()
{

}

ASpark::~ASpark()
{

}

void ASpark::update(double dt)
{
    AParticleSystem::update(dt);

    // TODO: for each particle, if the particle position is less than zero, modify its velocity so it bounces
}

//---------------------

ARocket::ARocket()
{
}

ARocket::~ARocket()
{
}

void ARocket::drawOpenGL()
{
    double alpha = atan2(mVel[1], mVel[0]) * 180.0 / 3.14;
    glColor4f(mColor[0], mColor[1], mColor[2], mAlpha);
    glPushMatrix();
    glTranslatef(mPos[0], mPos[1], mPos[2]);
    glScalef(mScale, mScale, mScale);
    glRotatef(90 + alpha, 0, 0, 1);
    glRotatef(90, 1, 0, 0);
    glutSolidCone(1.0, 3, 10, 10);
    glPopMatrix();
}

//---------------------
AFireworks::AFireworks() : mMode(ROCKET), mRocket(), mSparks(), mNumRings(5)
{
    for (int i = 0; i < mNumRings; i++)
    {
        mSparks.push_back(ASpark());
        mSparks[i].mInfinite = false;

        // TODO: Set default spark properties here
        mSparks[i].mEndColor = vec3(0, 0, 0);
        mSparks[i].mStartScale = 1.0;
        mSparks[i].mEndScale = 0.0;
        mSparks[i].mLifetime = 0.0;
        mSparks[i].mVelocityJitter = AJitter(0, 0);
    }
}

AFireworks::~AFireworks()
{

}

void AFireworks::reset(const vec3& color)
{
    mMode = ROCKET;

    // TODO: Set rocket values
    mRocket.mVel = vec3(0, 0, 0);
    mRocket.mPos = vec3(0, 0, 0);
    mRocket.mStartColor = mRocket.mEndColor = color;
    mRocket.mStartScale = mRocket.mEndScale = 1;
    mRocket.mStartAlpha = mRocket.mEndAlpha = 1;
    mRocket.mTtl = 0;

    for (int i = 0; i < mNumRings; i++)
    {
        mSparks[i].mStartColor = color;
        mSparks[i].reset();
    }
}

void AFireworks::update(double dt)
{
    vec3 forces(0, -9.8, 0);
    switch (mMode)
    {
    case ROCKET:
    {
        mRocket.update(dt, forces);
        if (!mRocket.isAlive())
        {
            mMode = EXPLODE;
            // init next phase
            for (int i = 0; i < mNumRings; i++)
            {
                mSparks[i].reset();

                // TODO: 
                // override positions & velocities of these sparks
                for (int j = 0; j < mSparks[i].mMaxParticles; j++)
                {
                    AParticle& p = mSparks[i].getParticle(j);
                    p.mPos = vec3(0, 0, 0);
                    p.mVel = vec3(0, 0, 0);
                }
            }
        }
        break;
    }
    case EXPLODE:
    {
        Mode m = DONE;
        for (int i = 0; i < mNumRings; i++)
        {
            mSparks[i].update(dt);
            if (mSparks[i].isAlive()) m = EXPLODE;
        }
        mMode = m;
        break;
    }

    case DONE:
    {
        break;
    }
    }
}

void AFireworks::drawOpenGL()
{
    switch (mMode)
    {
    case ROCKET: 
        mRocket.drawOpenGL(); 
        break;
    case EXPLODE:
        for (int i = 0; i < mNumRings; i++)
        {
            mSparks[i].drawOpenGL();
        }
        break;
    case DONE: 
        break;
    }
}
