#ifndef _BEHAVIOR_H
#define _BEHAVIOR_H

#include <string>
#include "aVector.h"
#include "aAnimatables.h"

// Behavior is an abstract base class for all behaviors
class BehaviorController; 
class Behavior
{
public:
    Behavior(const Behavior& orig);
    virtual ~Behavior() {}
    virtual const std::string& GetName() const;

    // Given an actor and behavior parameters, return a desired
    // velocity in world coordinates
    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor) = 0;

protected:
    Behavior(const char* name);
    std::string m_name;
};

class Obstacle
{
public:
    Obstacle() {}
    virtual ~Obstacle() {}
    double mRadius;
    AnimatableTransform mTransform;
};

class Seek : public Behavior
{
public:
    Seek(const AnimatableTransform& target);
    Seek(const Seek& orig);
    virtual ~Seek();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const AnimatableTransform& mTarget;
};

class Flee : public Behavior
{
public:
    Flee(const AnimatableTransform& target);
    Flee(const Flee& orig);
    virtual ~Flee();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const AnimatableTransform& mTarget;
};


class Avoid : public Behavior
{
public:
    Avoid(const AnimatableTransform& target, const std::vector<Obstacle>& obstacles);
    Avoid(const Avoid& orig);
    virtual ~Avoid();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:

    const AnimatableTransform& mTarget;
    const std::vector<Obstacle>& mObstacles;
};

class Separation : public Behavior
{
public:
    Separation(const AnimatableTransform& target, const std::vector<BehaviorController>& agents);
    Separation(const Separation& orig);
    virtual ~Separation();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const std::vector<BehaviorController>& mAgents;
    const AnimatableTransform& mTarget;
};

class Cohesion : public Behavior
{
public:
    Cohesion(const std::vector<BehaviorController>& agents);
    Cohesion(const Cohesion& orig);
    virtual ~Cohesion();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const std::vector<BehaviorController>& mAgents;
};

class Alignment : public Behavior
{
public:
    Alignment(const AnimatableTransform& target, const std::vector<BehaviorController>& agents);
    Alignment(const Alignment& orig);
    virtual ~Alignment();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const std::vector<BehaviorController>& mAgents;
    const AnimatableTransform& mTarget;
};

class Arrival : public Behavior
{
public:
    Arrival(const AnimatableTransform& target);
    Arrival(const Arrival& orig);
    virtual ~Arrival();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:

    const AnimatableTransform& mTarget;
};

class Departure : public Behavior
{
public:
    Departure(const AnimatableTransform& target);
    Departure(const Departure& orig);
    virtual ~Departure();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const AnimatableTransform& mTarget;
};

class Wander : public Behavior
{
public:
    Wander();
    Wander(const Wander& orig);
    virtual ~Wander();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

public:
    // the current direction
    vec3 mWander;
};


class Flocking : public Behavior
{
public:
    Flocking(const AnimatableTransform& target, const std::vector<BehaviorController>& agents);
    Flocking(const Flocking& orig);
    virtual ~Flocking();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const AnimatableTransform& mTarget;
    const std::vector<BehaviorController>& mAgents;
};

class Leader : public Behavior
{
public:
    Leader(const AnimatableTransform& target, std::vector<BehaviorController>& agents);
    Leader(const Leader& orig);
    virtual ~Leader();

    virtual vec3 calculateDesiredVelocity(const BehaviorController& actor);

protected:
    const AnimatableTransform& mTarget;
    std::vector<BehaviorController>& mAgents;
};


#endif
