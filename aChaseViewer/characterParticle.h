#include "aVector.h"
#include "aBVHController.h"

// This class tracks character position and velocity, and draws the char
class CharacterParticle
{
public:
	CharacterParticle(vec3 pos, vec3 vel);
	~CharacterParticle();
	vec3 pos;
	vec3 vel;
};