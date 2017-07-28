#pragma once

#include "aBasicViewer.h"
#include "aMotionBlender.h"
#include "aCharacterDrawer.h"

class BlendViewer : public ABasicViewer
{
public:
    BlendViewer();
    virtual ~BlendViewer();
    virtual void loadMotion1(const std::string& filename);
    virtual void loadMotion2(const std::string& filename);
    virtual void blend();

protected:

    virtual void initializeGui();
    virtual void draw3DView();
    virtual void onTimer(int value);
    virtual void load(const std::string& filename);
    virtual void onLoadMotion1();
    virtual void onLoadMotion2();

    static void TW_CALL onBlendCb(void *clientData);
    static void TW_CALL onLoadMotion1Cb(void *clientData);
    static void TW_CALL onLoadMotion2Cb(void *clientData);
    static void TW_CALL onSetKeyCb(const void *value, void *clientData);
    static void TW_CALL onGetKeyCb(void *value, void *clientData);

protected:
    std::string mFilename1;
    std::string mFilename2;
    int mNumKeys1;
    int mNumKeys2;
    MotionBlender mMotionBlender;
    int mNumKeys;
    int mCurrentKey;
    CharacterDrawer mDrawer;

    TwBar *mBlendBar;
    int mMotionKeyId1;
    int mMotionKeyId2;
    int mNumBlendKeys;
};
