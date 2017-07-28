#pragma once

#include <vector>
#include "aBasicViewer.h"
#include "aBVHController.h"
#include "aCharacterDrawer.h"
#include "characterParticle.h"

class BVHViewer : public ABasicViewer
{
public:
    BVHViewer();
    virtual ~BVHViewer();
    virtual void loadMotion(const std::string& filename);

protected:

    virtual void initializeGui();
    virtual void draw3DView();
    virtual void onStepForward();
    virtual void onStepBack();
    virtual void onTimer(int value);
    virtual void load(const std::string& filename);

protected:
    std::vector<CharacterParticle*> characters;
    BVHController mBVHController;
    CharacterDrawer mDrawer;
    int NUM_CHARS;
    vec3 leader_colors;
    vec3 follower_colors;
};
