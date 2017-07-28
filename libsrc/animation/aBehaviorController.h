#ifndef BehaviorController_H_
#define BehaviorController_H_

#include "aVector.h"
#include "aAnimatables.h"

class Behavior;
class BehaviorController
{
public:
    BehaviorController();
    virtual ~BehaviorController();

    void reset();
    virtual void update(double fTime);
    virtual void setBehavior(Behavior* behavior);
    virtual Behavior* getBehavior() const { return mBehavior; }

    void setActive(bool b) { mActive = b; }
    bool isActive() const { return mActive; }

    const AnimatableTransform& getTransform() const { return mTransform;  }
    vec3 getPosition() const { return mTransform.getLocalTranslation(); }
    vec3 getDesiredVelocity() const { return mDVel; }
    vec3 getVelocity() const { return mVel;  }

protected:

    //Sense the environment according to different behaviors
    virtual void sense(double fTime);

    //Agent act: performing EulerStep for this particle
    virtual void act(double fTime);
    virtual void findDerivative(double fTime);
    virtual void updateState(double fTime);

    //Agent control
    virtual void control(double fTime);

protected:
    AnimatableTransform mTransform;
    bool mActive;
    Behavior* mBehavior;

    // 1D state: speed and heading around UP
    enum {POS, ORI, VEL, AVEL};
    double state[4];
    double derivative[4];

    // Input vector: 1D force and torque
    double force;
    double torque;

    // Control input: 1D speed and heading
    enum {USPEED, UHEADING};
    double vd;
    double thetad;

    vec3 mDVel; // desired velocity
    vec3 mVel; // current velocity

public:
    static double gKNeighborhood;
    static double gAgentRadius;

    static double gMass;
    static double gInertia;
    static double gMaxSpeed;
    static double gMaxAngularSpeed;
    static double gMaxForce;
    static double gMaxTorque;

    //Velocity control: f = m * Kv0 * (vd - v)
    static double gVelKv;  // Velocity 

    //Heading control: tau = I * ( -Kv1 * thetaDot - Kp1 * theta + Kp1 * thetad)
    static double gOriKv;  // Orientation
    static double gOriKp;

    //Behavior settings. See comments in cpp file for details
    static double KArrival;
    static double KDeparture;
    static double KNoise;
    static double KWander;
    static double KAvoid;
    static double TAvoid;
    static double RNeighborhood;
    static double KSeparation;
    static double KAlignment;
    static double KCohesion;
};

#endif
