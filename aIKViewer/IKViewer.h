#ifndef IKViewer_H_
#define IKViewer_H_

#include <string>
#include "aBasicViewer.h"
#include "aAnimatables.h"
#include "aIKController.h"
#include "aCharacterDrawer.h"

using namespace std;
class IKViewer : public ABasicViewer
{
public:
    IKViewer();
    virtual ~IKViewer();
    virtual bool loadMotion(const std::string& filename);

protected:

    virtual void initializeGui();
    virtual void draw3DView();
    virtual void drawOverlay();
    virtual void onTimer(int value);
    virtual void onSelectIKJoint(int selectedJoint);
    virtual void loadIKJoints();
    virtual void updateIK();
    virtual void reset();
    virtual void onMouseMotion(int pX, int pY);
    virtual void onMouse(int button, int state, int x, int y);

    static void TW_CALL ResetIKCb(void* clientData);
    static void TW_CALL SelectIKCb(void* clientData);
    static void TW_CALL onSetIKCb(const void *value, void *clientData);
    static void TW_CALL onGetIKCb(void *value, void *clientData);

    static void TW_CALL onSetGoalXCb(const void *value, void *clientData);
    static void TW_CALL onGetGoalXCb(void *value, void *clientData);
    static void TW_CALL onSetGoalYCb(const void *value, void *clientData);
    static void TW_CALL onGetGoalYCb(void *value, void *clientData);
    static void TW_CALL onSetGoalZCb(const void *value, void *clientData);
    static void TW_CALL onGetGoalZCb(void *value, void *clientData);

protected:
    Actor mActor;
    IKController mIKController;
    CharacterDrawer mDrawer;
    vec3 mGoalPosition;
    int mSelectedJoint;
    int mIKChainSize;
    bool mSelectedRecticle;
    int mLastIKX, mLastIKY;

    TwBar *mPoseEditBar;
    struct EffectorData { IKViewer* viewer; int jointid; std::string name; };
    std::vector<EffectorData*> mEffectorData;

    enum IKType { ANALYTIC, CCD } mType;
    TwType ikType;
    string mFilename;
};

#endif
