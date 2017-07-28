#ifndef CHARACTERDRAWER_H_
#define CHARACTERDRAWER_H_

#include "aAnimatables.h"

class CharacterDrawer 
{
public:
	CharacterDrawer();
	virtual ~CharacterDrawer();
	virtual void draw(const Actor& skeleton);

protected:

    void drawSphereLimb(const vec3& startPosition, const vec3& endPosition);
    void drawJoint(const Transform& globalTransform);
    void glApplyRotation(const mat3& rotation);
    void setMaterial(const vec3& colorVec);
    void drawBodyParts(AnimatableTransform* currentJoint);

public:
    static bool gDraw;
    vec3 chosenColor;
};

#endif
