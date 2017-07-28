#include "characterParticle.h"
#include "aSplineVec3.h"

using namespace std;

CharacterParticle::CharacterParticle(vec3 pos, vec3 vel) {
	this->pos = pos;
	this->vel = vel;
};

CharacterParticle::~CharacterParticle() {
};