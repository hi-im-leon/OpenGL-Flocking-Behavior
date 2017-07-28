#include "aJitter.h"

double AJitterVal(const AJitter& range)
{
    // TODO: return a random number between range
    return range.first;
}

vec3 AJitterVec(const AJitter& range)
{
    return vec3(AJitterVal(range), AJitterVal(range), AJitterVal(range));
}
