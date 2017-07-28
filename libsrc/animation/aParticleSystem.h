#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <vector>
#include "aAnimatables.h"
#include "aVector.h"
#include "aJitter.h"

class AParticle;
class AParticleSystem 
{
public:
    AParticleSystem();
    virtual ~AParticleSystem();

    virtual void setRoot(AnimatableTransform& transform) { mRoot = &transform;  }
    virtual void update(double dt);
    virtual void drawOpenGL(); // for debugging
    virtual void reset();
    virtual bool isAlive(); // is playing once, return whether any particles are active
    AParticle& getParticle(int idx) { return mParticles[idx];  }
    const AParticle& getParticle(int idx) const { return mParticles[idx]; }
 
protected:
    std::vector<AParticle> mParticles;
    AnimatableTransform* mRoot; // attachment for this particle system

public:

    // particle configuration
    bool mInfinite; // play once, or continue respawning particles?
    int mMaxParticles;
    double mLifetime;
    vec3 mGravity;
    vec3 mStartColor, mEndColor;
    vec3 mStartPos;
    vec3 mStartVel;
    double mStartAlpha, mEndAlpha;  
    double mStartScale, mEndScale;

    AJitter mScaleJitter;
    AJitter mColorJitter;
    AJitter mPositionJitter;
    AJitter mVelocityJitter;
};

class AParticle
{
public:
    AParticle();
    virtual ~AParticle();
    virtual void init(const AParticleSystem& parent);
    virtual void update(double dt, const vec3& externalForces);
    virtual bool isAlive() const;

    double mTtl, mLifetime; //time to live
    double mMass;
    vec3 mPos;
    vec3 mVel;
    vec3 mColor, mStartColor, mEndColor;
    double mScale, mStartScale, mEndScale;
    double mAlpha, mStartAlpha, mEndAlpha;
};


#endif 
