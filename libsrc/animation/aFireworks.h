#ifndef AFireworks_H_
#define AFireworks_H_

#include "aParticleSystem.h"

class ASpark : public AParticleSystem
{
public:
    ASpark();
    virtual ~ASpark();
    virtual void update(double dt);
};

class ARocket : public AParticle
{
public:
    ARocket(); 
    virtual ~ARocket();
    virtual void drawOpenGL();
};

// Implements a rocket and fireworks using particle systems organized with a FSM
class AFireworks
{
public:
    AFireworks();
    virtual ~AFireworks();
    virtual void update(double dt);
    virtual void drawOpenGL(); 
    virtual void reset(const vec3& color);

protected:
    enum Mode {ROCKET, EXPLODE, DONE} mMode;
    ARocket mRocket;
    std::vector<ASpark> mSparks;

public:
    int mNumRings;
};

#endif