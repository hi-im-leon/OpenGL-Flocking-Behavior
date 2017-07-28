#pragma once

#include "aBasicViewer.h"
#include "aBVHController.h"
#include "aCharacterDrawer.h"

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
    BVHController mBVHController;
    CharacterDrawer mDrawer;
    int NUM_CHARS;
    vec3 * colors;
};
