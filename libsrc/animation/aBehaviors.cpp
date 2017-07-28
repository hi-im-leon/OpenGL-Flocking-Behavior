#include "aBehaviors.h"
#include "aBehaviorController.h"
#include <math.h>

Behavior::Behavior(const char* name) : m_name(name)
{
}

Behavior::Behavior(const Behavior& orig) : m_name(orig.m_name)
{
}

const std::string& Behavior::GetName() const
{
    return m_name;
}

Cohesion::Cohesion(const std::vector<BehaviorController>& agents) :
Behavior("cohesion"), mAgents(agents)
{
}

Cohesion::~Cohesion()
{
}

Cohesion::Cohesion(const Cohesion& orig) : Behavior(orig), mAgents(orig.mAgents)
{
}

//Cohesion behavior
//  CAgent::agents[i] gives the pointer to the ith agent in the environment
//  Cohesion settings are in CAgent::RNeighborhood and CAgent::KCohesion
// Given the actor, return a desired velocity in world coordinates
// Cohesion moves actors towards the center of a group of agents
vec3 Cohesion::calculateDesiredVelocity(const BehaviorController& actor)
{
    for (unsigned int i = 0; i < mAgents.size(); i++)
    {
    }

    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//----

Departure::Departure(const AnimatableTransform& target) : Behavior("departure"), mTarget(target)
{
}

Departure::~Departure()
{
}

Departure::Departure(const Departure& orig) :
Behavior(orig), mTarget(orig.mTarget)
{
}

//Departure behavior
//  Global goal position is in goal
//  Agent's global position is in GPos
//  Departure setting is in KDeparture
// Given the actor, return a desired velocity in world coordinates
// Departure calculates a repelent velocity based on the actor's 
// distance from the target
vec3 Departure::calculateDesiredVelocity(const BehaviorController& actor)
{
    //vec3 targetPos = mTarget.getLocalTranslation();
    //vec3 actorPos = actor.getPosition();

    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//---

Flee::Flee(const AnimatableTransform& target) : Behavior("flee"), mTarget(target)
{
}

Flee::~Flee()
{
}

Flee::Flee(const Flee& orig) :
Behavior(orig), mTarget(orig.mTarget)
{
}

//Flee behavior
//  Global goal position is in mTarget
//  Agent's global position is in actor.getPosition()
// Given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
vec3 Flee::calculateDesiredVelocity(const BehaviorController& actor)
{
    //vec3 targetPos = mTarget.getLocalTranslation();
    //vec3 actorPos = actor.getPosition();

    vec3 desiredVel(0, 0, 0);
    return desiredVel;

}

//----

Flocking::Flocking(const AnimatableTransform& target, const std::vector<BehaviorController>& agents) :
    Behavior("flocking"), mTarget(target), mAgents(agents)
{
}

Flocking::Flocking(const Flocking& orig) :
    Behavior(orig), mTarget(orig.mTarget), mAgents(orig.mAgents)
{
}

Flocking::~Flocking()
{
}

// Flocking behavior
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector
// Given the actor, return a desired velocity in world coordinates
// Flocking combines separation, cohesion, and alignment
vec3 Flocking::calculateDesiredVelocity(const BehaviorController& actor)
{
    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//---

Leader::Leader(const AnimatableTransform& target, std::vector<BehaviorController>& agents) :
Behavior("leader"), mTarget(target), mAgents(agents)
{
}

Leader::Leader(const Leader& orig) :
Behavior(orig), mTarget(orig.mTarget), mAgents(orig.mAgents)
{
}

Leader::~Leader()
{
}

//	Flocking behavior
//  Utilize the Separation, Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always the first agent in CAgent::agents
// Given the actor, return a desired velocity in world coordinates
// If the actor is the leader, move towards the target; otherwise, 
// follow the leader without bunching together

vec3 Leader::calculateDesiredVelocity(const BehaviorController& actor)
{
    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//--


Seek::Seek(const AnimatableTransform& target) : Behavior("seek"), mTarget(target)
{
}

Seek::~Seek()
{
}

Seek::Seek(const Seek& orig) : Behavior(orig), mTarget(orig.mTarget)
{
}

//Seek behavior
//  Global goal position is in mTarget
//  Agent's global position is in actor.getPosition()
// Given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
vec3 Seek::calculateDesiredVelocity(const BehaviorController& actor)
{
    //vec3 targetPos = mTarget.getLocalTranslation();
    //vec3 actorPos = actor.getPosition();

    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//-- 

Separation::Separation(const AnimatableTransform& target, const std::vector<BehaviorController>& agents) :
Behavior("separation"), mAgents(agents), mTarget(target)
{
}

Separation::~Separation()
{
}

Separation::Separation(const Separation& orig) :
Behavior(orig), mAgents(orig.mAgents), mTarget(orig.mTarget)
{
}

///Separation behavior
//  CAgent::agents[i] gives the pointer to the ith agent in the environment
//  Separation settings are in CAgent::RNeighborhood and CAgent::KSeperate

// Given the actor, return a desired velocity in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood of the agent
vec3 Separation::calculateDesiredVelocity(const BehaviorController& actor)
{
    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//---

Wander::Wander() :
Behavior("wander"),
mWander(1.0, 0.0, 0.0)
{
}

Wander::~Wander()
{
}

Wander::Wander(const Wander& orig) : Behavior(orig), mWander(orig.mWander)
{
}

// VWander is in vWander
//  V0(nominal velocity) is in v0
//  Wander setting is in KWander
// Given the actor, return a desired velocity in world coordinates
// Wander returns a velocity whose direction changes at random
vec3 Wander::calculateDesiredVelocity(const BehaviorController& actor)
{
    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//-----

Alignment::Alignment(const AnimatableTransform& target, const std::vector<BehaviorController>& agents) :
Behavior("alignment"), mAgents(agents), mTarget(target)
{
}

Alignment::~Alignment()
{
}

Alignment::Alignment(const Alignment& orig) :
Behavior(orig), mAgents(orig.mAgents), mTarget(orig.mTarget)
{
}

//Alignment behavior
// CAgent::agents[i] gives the pointer to the ith agent in the environment
//  Alignment settings are in CAgent::RNeighborhood and CAgent::KAlign
// Given the actor, return a desired velocity in world coordinates
// Alignment returns an average velocity of all active agents
vec3 Alignment::calculateDesiredVelocity(const BehaviorController& actor)
{
    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}
//------------------------------------------------------------------------------------------------------

Arrival::Arrival(const AnimatableTransform& target) :
Behavior("arrival"),
mTarget(target)
{
}

Arrival::Arrival(const Arrival& orig) :
Behavior(orig), mTarget(orig.mTarget)
{
}

Arrival::~Arrival()
{
}

//Arrival behavior
//  Global goal position is in goal
//  Agent's global position is in GPos
//  Arrival setting is in CAgent::KArrival
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a velocity whose speed is proportional to the actors distance
// from the target
vec3 Arrival::calculateDesiredVelocity(const BehaviorController& actor)
{
    //vec3 targetPos = mTarget.getLocalTranslation();
    //vec3 actorPos = actor.getPosition();

    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//-----

Avoid::Avoid(const AnimatableTransform& target, const std::vector<Obstacle>& obstacles) :
    Behavior("avoid"), mTarget(target), mObstacles(obstacles)
{
}

Avoid::Avoid(const Avoid& orig) :
    Behavior(orig), mTarget(orig.mTarget), mObstacles(orig.mObstacles)
{
}

Avoid::~Avoid()
{
}

//Avoid behavior
//  Obstacles are in mObstacles and have class type Obstacle
//  Agent bounding sphere radius is in CAgent::radius
//  Avoidance settings are in CAgent::TAvoid and CAgent::KAvoid
// Given the actor, return a desired velocity in world coordinates
// If an actor is near an obstacle, avoid adds either a tangential or
// normal response velocity
vec3 Avoid::calculateDesiredVelocity(const BehaviorController& actor)
{
    vec3 desiredVel(0, 0, 0);
    return desiredVel;
}

//----

