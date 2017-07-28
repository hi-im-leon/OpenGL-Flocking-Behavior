#include "aBehaviorController.h"
#include "aBehaviors.h"
#include "aVector.h"
#include "aRotation.h"
#include <algorithm>
#include <string.h>

using namespace std;
#define Truncate(a, b, c) (a = std::max<double>(std::min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1.0;
double BehaviorController::gMaxAngularSpeed = 1.0;
double BehaviorController::gMaxForce = 1.0;
double BehaviorController::gMaxTorque = 1.0;
double BehaviorController::gKNeighborhood = 1.0;
double BehaviorController::gOriKv = 1.0;  
double BehaviorController::gOriKp = 1.0;
double BehaviorController::gVelKv = 1.0;  // Velocity 
double BehaviorController::gAgentRadius = 1.0;
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1;
double BehaviorController::KDeparture = 1;
double BehaviorController::KNoise = 1;
double BehaviorController::KWander = 1.0;
double BehaviorController::KAvoid = 1.0;
double BehaviorController::TAvoid = 1.0;
double BehaviorController::KSeparation = 1.0;
double BehaviorController::KAlignment = 1.0;
double BehaviorController::KCohesion = 1.0;

const double M2_PI = M_PI * 2.0;
inline void ClampAngle(double& angle)
{
    while (angle > M_PI)
    {
        angle -= M2_PI;
    }
    while (angle < -M_PI)
    {
        angle += M2_PI;
    }
}

BehaviorController::BehaviorController() :
    mTransform(), mActive(true), mBehavior(0)
{
    reset();
}

BehaviorController::~BehaviorController()
{
    mBehavior = 0;
}

void BehaviorController::reset()
{
    mVel.set(0, 0, 0); 
    mDVel.set(0, 0, 0);

    vec3 startPos(((double)rand()) / RAND_MAX, 
                  ((double)rand()) / RAND_MAX, 
                  ((double)rand()) / RAND_MAX);
    startPos = startPos - vec3(0.5, 0.5, 0.5);

    startPos[1] = 0; // assume Y up

    mTransform.setLocalTranslation(startPos * 500);

    memset(state, 0, 4 * sizeof(double));
    memset(derivative, 0, 4 * sizeof(double));
    force = 0;
    torque = 0;
    thetad = 0;
    vd = 0;
}

// Given an actor, update its movement through forces and torques.
// This function obtains a desired velocity.  With the desired velocity
// we can then calculate the corresponding torque and force that will
// achieve it over time
void BehaviorController::update(double dt)
{
    if (isActive())
    {
        sense(dt);
        control(dt);
        act(dt);
    }
}

void BehaviorController::sense(double dt)
{
    if (mBehavior)
    {
        mDVel = mBehavior->calculateDesiredVelocity(*this);
        mDVel[1] = 0;

        vd = mDVel.Length();

        thetad = 0.0;
        if (vd > 0.0001)
        {
            if (abs(mDVel[0]) < 0.0001)
            {
                if (mDVel[2] > 0) thetad = M_PI / 2.0;
                else thetad = M_PI / -2.0;
            }
            else thetad = atan2(mDVel[2], mDVel[0]);
        }
        ClampAngle(thetad);
    }
}

void BehaviorController::act(double dt)
{
    findDerivative(dt);
    updateState(dt);
}

// Compute derivative vector given input and state vectors
//  This function sets derive vector to appropriate values after being called
void BehaviorController::findDerivative(double dt)
{
    derivative[POS] = 0;
    derivative[ORI] = 0;
    derivative[VEL] = 0;
    derivative[AVEL] = 0;
}

// Update the state vector given derivative vector
//  Compute global position and store it in GPos
//  Perform validation check to make sure all values are within MAX values
void BehaviorController::updateState(double dt)
{
    // TODO: Set state
    state[POS] = 0;
    state[ORI] += 0;
    state[VEL] += 0;
    state[AVEL] += 0;

    // TODO: Clamp values

    mVel[0] = state[VEL] * cos(state[ORI]);
    mVel[1] = 0;
    mVel[2] = state[VEL] * sin(state[ORI]);

    vec3 dir = mVel;
    dir.Normalize();
    vec3 up(0, 1, 0);
    vec3 right = up.Cross(dir);
    mat3 rot(right, up, dir);
    mTransform.setLocalRotation(rot.Transpose());
    mTransform.setLocalTranslation(mTransform.getLocalTranslation() + mVel);
}

// You should apply the control rules given desired velocity vd and desired orientation thetad.
//  Velocity control : force = m * Kv0 * (vd - v)
//  Heading control : torque = I * (-Kv1 * thetaDot -Kp1 * (thetad - theta))
void BehaviorController::control(double dt)
{
    force = 0;
    torque = 0;
}

void BehaviorController::setBehavior(Behavior* behavior)
{
    mBehavior = behavior;
}
