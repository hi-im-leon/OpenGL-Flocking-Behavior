#include <string>
#include "BVHViewer.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>

BVHViewer::BVHViewer() : mBVHController(), mDrawer()
{
    this->NUM_CHARS = 100;
    this->colors = new vec3[NUM_CHARS];
    
    for(int i = 0; i < NUM_CHARS; i++) {
        colors[i] = vec3(rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX);
    } 
}

BVHViewer::~BVHViewer()
{
}

void BVHViewer::load(const std::string& filename)
{
    loadMotion(filename);
}

void BVHViewer::loadMotion(const std::string& filename)
{
    mBVHController.load(filename);
    mFilename = pruneName(filename);
}

void BVHViewer::initializeGui()
{
    ABasicViewer::initializeGui();
}

void BVHViewer::onStepBack()
{
    double dt = 1.0/mBVHController.getFramerate();
    mCurrentFrame = mCurrentFrame - 1;
    if (mCurrentFrame < 0) mCurrentFrame = mBVHController.getNumKeys() - 1;
    mCurrentTime = mCurrentFrame * dt;
}

void BVHViewer::onStepForward()
{
    double dt = 1.0/mBVHController.getFramerate();
    mCurrentFrame = (mCurrentFrame + 1) % mBVHController.getNumKeys();
    mCurrentTime = mCurrentFrame * dt;
}

void BVHViewer::onTimer(int value)
{
    ABasicViewer::onTimer(value);
    mBVHController.update(mCurrentTime);
    mCurrentFrame = mBVHController.getFrame(mCurrentTime);
}

void BVHViewer::draw3DView()
{
    glViewport(0, 0, (GLsizei)mWindowWidth, (GLsizei)mWindowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // Draw the front face only, except for the texts and lights.
    glEnable(GL_LIGHTING);

    GLfloat fogColor[4] = {1.0, 0, 0, 1};
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogfv(GL_FOG_COLOR, fogColor);
    glClearColor(fogColor[0], fogColor[1], fogColor[2], 1);

    glHint(GL_FOG_HINT, GL_DONT_CARE);
    glFogf(GL_FOG_DENSITY, 0.8);
    glFogf(GL_FOG_START, 0);             // Fog Start Depth
    glFogf(GL_FOG_END, 500.0);               // Fog End Depth
    glEnable(GL_FOG);                   // Enables GL_FOG

    // Set the view to the current camera settings.
    mCamera.draw();

    GLfloat pos[4];
    pos[0] = mCamera.getPosition()[0];
    pos[1] = mCamera.getPosition()[1];
    pos[2] = mCamera.getPosition()[2];
    pos[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_POSITION, pos);


    for (int i = 0; i < NUM_CHARS; i++) {
        glPushMatrix();
        glTranslatef();
        glScalef(0.5, 0.5, 0.5);
        mDrawer.chosenColor = colors[i];    
        mDrawer.draw(mBVHController.getSkeleton());
        glPopMatrix();
    }
    
    glDisable(GL_LIGHTING);
    displayGrid();
}

