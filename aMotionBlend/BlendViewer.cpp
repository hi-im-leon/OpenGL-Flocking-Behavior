#include <string>
#include "BlendViewer.h"
#include "aOsUtils.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>

BlendViewer::BlendViewer() : 
   mNumKeys1(0), 
   mNumKeys2(0), 
   mMotionBlender(), 
	mNumKeys(0),
	mCurrentKey(0),
   mMotionKeyId1(0), 
   mMotionKeyId2(0), 
   mNumBlendKeys(10)
{
    mFilename = "Blend";
}

BlendViewer::~BlendViewer()
{
}

void BlendViewer::load(const std::string& filename)
{
    loadMotion1(filename);
}

void BlendViewer::loadMotion1(const std::string& filename)
{
    mMotionBlender.loadMotion1(filename);
    mNumKeys1 = mMotionBlender.getNumKeys1();
    mFilename1 = pruneName(filename);
    mMotionKeyId1 = mNumKeys1-mNumBlendKeys;
    mNumKeys = mMotionBlender.getNumKeys();
    mCurrentKey = 0;
    mCurrentTime = 0;
}

void BlendViewer::loadMotion2(const std::string& filename)
{
    mMotionBlender.loadMotion2(filename);
    mNumKeys2 = mMotionBlender.getNumKeys2();
    mFilename2 = pruneName(filename);
    mMotionKeyId2 = 0;
    mNumKeys = mMotionBlender.getNumKeys();
    mCurrentKey = 0;
    mCurrentTime = 0;
}

void BlendViewer::onLoadMotion1()
{
    std::string filename = PromptToLoad();
    loadMotion1(filename);
}

void BlendViewer::onLoadMotion2()
{
    std::string filename = PromptToLoad();
    loadMotion2(filename);
}

void BlendViewer::blend()
{
    std::cout << "BLEND " << mMotionKeyId1 << " " << 
		  mMotionKeyId2 << " " << mNumBlendKeys << std::endl;
    mMotionBlender.blend(mMotionKeyId1, mMotionKeyId2, mNumBlendKeys);
    mNumKeys = mMotionBlender.getNumKeys();
}

void BlendViewer::initializeGui()
{
    ABasicViewer::initializeGui();
    TwDefine(" 'File controls' size='200 200' position='5 5' iconified=true fontresizable=false alpha=200");

    TwAddVarCB(mPlayerBar, "Key", TW_TYPE_INT32, onSetKeyCb, onGetKeyCb, this, " step=1");
    TwAddVarRO(mPlayerBar, "NumKeys", TW_TYPE_INT32, &mNumKeys, "");

    mBlendBar = TwNewBar("Blend controls");
    TwDefine(" 'Blend controls' size='200 250' position='5 185' iconified=false fontresizable=false alpha=200");
    TwAddVarRO(mBlendBar, "Motion 1", TW_TYPE_STDSTRING, &mFilename1, "");
    TwAddVarRO(mBlendBar, "NumKeys 1", TW_TYPE_INT32, &mNumKeys1, "");
    TwAddButton(mBlendBar, "LoadMotion1Btn", onLoadMotion1Cb, this, " label='Load Motion1'");
    TwAddSeparator(mBlendBar, NULL, "");
    TwAddVarRO(mBlendBar, "Motion 2", TW_TYPE_STDSTRING, &mFilename2, "");
    TwAddVarRO(mBlendBar, "Duration 2", TW_TYPE_INT32, &mNumKeys2, "");
    TwAddButton(mBlendBar, "LoadMotion2Btn", onLoadMotion2Cb, this, " label='Load Motion2'");
    TwAddSeparator(mBlendBar, NULL, "");
    TwAddVarRW(mBlendBar, "Key 1", TW_TYPE_INT32, &mMotionKeyId1, " group='Blend'");
    TwAddVarRW(mBlendBar, "Key 2", TW_TYPE_INT32, &mMotionKeyId2, " group='Blend'");
    TwAddVarRW(mBlendBar, "Num Keys", TW_TYPE_INT32, &mNumBlendKeys, " group='Blend'");
    TwAddButton(mBlendBar, "BlendBtn", onBlendCb, this, " label='Blend' group='Blend'");
}

void BlendViewer::onTimer(int value)
{
    ABasicViewer::onTimer(value);
    mMotionBlender.update(mCurrentTime);
}

void BlendViewer::draw3DView()
{
    glViewport(0, 0, (GLsizei)mWindowWidth, (GLsizei)mWindowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // Draw the front face only, except for the texts and lights.
    glEnable(GL_LIGHTING);

    // Set the view to the current camera settings.
    mCamera.draw();

    GLfloat pos[4];
    pos[0] = mCamera.getPosition()[0];
    pos[1] = mCamera.getPosition()[1];
    pos[2] = mCamera.getPosition()[2];
    pos[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    glDisable(GL_TEXTURE_2D);
    mDrawer.draw(mMotionBlender.getSkeleton());

    glDisable(GL_LIGHTING);
    displayGrid();
}

void TW_CALL BlendViewer::onBlendCb(void *usrData)
{
    ((BlendViewer*)usrData)->blend();
}


void TW_CALL BlendViewer::onSetKeyCb(const void *value, void *clientData)
{
    BlendViewer* viewer = ((BlendViewer*)clientData);
    int v = *(const int *)value;  // for instance
    if (viewer->mNumKeys == 0) return;

    if (v < 0) v = viewer->mNumKeys + v;
    else v = v % viewer->mNumKeys;

    viewer->mCurrentKey = v;
    double fps = viewer->mMotionBlender.getFramerate();
    if (fps > 0.00001)
    {
        viewer->mCurrentTime = v / fps;
    }

    //std::cout << "SET FRAME BUTTON PRESSED " << v << " " << std::endl;
    glutPostRedisplay();
}

void TW_CALL BlendViewer::onGetKeyCb(void *value, void *clientData)
{
    BlendViewer* viewer = ((BlendViewer*)clientData);
    *static_cast<int *>(value) = viewer->mCurrentKey;
}

void TW_CALL BlendViewer::onLoadMotion1Cb(void *usrData)
{
    ((BlendViewer*)usrData)->onLoadMotion1();
}

void TW_CALL BlendViewer::onLoadMotion2Cb(void *usrData)
{
    ((BlendViewer*)usrData)->onLoadMotion2();
}
