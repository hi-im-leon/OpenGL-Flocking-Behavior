#include <string>
#include "IKSimple.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <math.h>
#include <algorithm>
#include <AntTweakBar.h>
#include <iostream>
#include "aBVHController.h"

using namespace std;

IKSimple::IKSimple() : 
   mDrawer(),
   mGoalPosition()
{
   mat3 identity;
   identity.FromEulerAnglesXYZ(vec3(0,0,0));

   Joint* shoulder = new Joint("Shoulder");
   mActor.addTransform(shoulder, true);
   shoulder->setLocalTranslation(vec3(0,0,0));
   shoulder->setNumChannels(6);
   shoulder->setRotationOrder("xyz");
   shoulder->setLocalRotation(identity);

   Joint* elbow = new Joint("Elbow");
   mActor.addTransform(elbow, false);
   elbow->setLocalTranslation(vec3(100,0,0));
   elbow->setNumChannels(3);
   elbow->setRotationOrder("xyz");
   elbow->setLocalRotation(identity);

   Joint* wrist = new Joint("Wrist");
   mActor.addTransform(wrist, false);
   wrist->setLocalTranslation(vec3(80,0,0));
   wrist->setNumChannels(3);
   wrist->setRotationOrder("xyz");
   wrist->setLocalRotation(identity);

   AnimatableTransform::Attach(shoulder, elbow);
   AnimatableTransform::Attach(elbow, wrist);

   mActor.updateTransforms();

   mGoalPosition = wrist->getGlobalTranslation();
   mSelectedJoint = 2;
}

IKSimple::~IKSimple()
{
}

void IKSimple::initializeGui()
{
    ABasicViewer::initializeGui();

    TwDefine(" 'File controls' size='200 300' position='5 185' iconified=true fontresizable=false alpha=200");
    TwDefine(" 'Player controls' size='200 175' position='5 5' iconified=true fontresizable=false alpha=200");

    mPoseEditBar = TwNewBar("Edit Pose");
    TwDefine(" 'Edit Pose' size='200 500' position='5 5' iconified=false fontresizable=false alpha=200");
    TwAddButton(mPoseEditBar, "Reset Pose", ResetIKCb, this, " ");
    TwAddVarCB(mPoseEditBar, "X", TW_TYPE_DOUBLE, onSetGoalXCb, onGetGoalXCb, this, " group='Goal position'");
    TwAddVarCB(mPoseEditBar, "Y", TW_TYPE_DOUBLE, onSetGoalYCb, onGetGoalYCb, this, " group='Goal position'");
    TwAddVarCB(mPoseEditBar, "Z", TW_TYPE_DOUBLE, onSetGoalZCb, onGetGoalZCb, this, " group = 'Goal position'");

   loadIKJoints();
}

void IKSimple::reset()
{
    mat3 identity;
    identity.FromEulerAnglesXYZ(vec3(0,0,0));
    for (int i = 0; i < mActor.getNumTransforms(); i++)
	{
       mActor.getByID(i)->setLocalRotation(identity);
	}
    mActor.updateTransforms();
    mGoalPosition = mActor.getByID(2)->getGlobalTranslation();
    glutPostRedisplay();
}

void IKSimple::updateIK() 
{
    // Implement the simple two-link IK algorithm from class
    double length2, length3, r, phicosine, phi, theta_2z, theta_1z;
    Joint* root = mActor.getByID(0);
    Joint* elbow = mActor.getByID(1);
    Joint* wrist = mActor.getByID(2);
    length2 = elbow->getLocalTranslation().Length();
    length3 = wrist->getLocalTranslation().Length();
    r = (mGoalPosition - root->getGlobalTranslation()).Length();
    phicosine = (r*r - length2*length2 -length3*length3)/(-2*length2*length3);
    if (phicosine > 1) {phicosine = 1;}
    if (phicosine < -1) {phicosine = -1;}
    phi = acos(phicosine);
    theta_2z = phi - M_PI;
    if (r == 0) {
        theta_1z = 0;
    } else {
        theta_1z = asin((length3*sin(phi))/r);
    }
    mat3 angle1, angle2;
    angle1.FromEulerAnglesZXY(vec3(0, 0, theta_1z));
    angle2.FromEulerAnglesZXY(vec3(0, 0, theta_2z));
    mat3 heading, pitch;
    double beta = atan2(-mGoalPosition[2], mGoalPosition[0]);
    double sinstuff = mGoalPosition[1]/r;
    if (sinstuff > 1) {
        sinstuff = 1;
    } else if (sinstuff < -1) {
        sinstuff = -1;
    }   
    double gamma = asin(sinstuff);
    // cout << "Debug Information Table " << "\n" << "------------------------\n" << 
    //     "phi: " << phi << " theta_2z: " << theta_2z << " theta_1z: " << theta_1z << "\n" <<
    //         "beta: " << beta << " gamma: " << gamma << "\n\n";
    heading.FromEulerAnglesYXZ(vec3(0, beta, 0));
    pitch.FromEulerAnglesZYX(vec3(0, 0, gamma));
    root->setLocalRotation(heading*pitch*angle1);
    elbow->setLocalRotation(angle2);
    mActor.updateTransforms();
}

void IKSimple::onTimer(int value)
{
    ABasicViewer::onTimer(value);
    updateIK();
}

void IKSimple::draw3DView()
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

    mDrawer.draw(mActor);

    glDisable(GL_LIGHTING);
    displayGrid();
}

void DrawCircle(const vec3& p)
{
    int vertices = 20;
    double r = 5.0;
    double tmpX, tmpY, tmpZ;
    double Angle, Angle0;

    Angle = -(2 * 3.14) / vertices;
    Angle0 = 0.0;

    tmpX = p[0];
    tmpY = p[1];
    tmpZ = p[2];

    glBegin(GL_LINE_LOOP);

    for (int i = 0; i < vertices; i++) {
        glVertex3f(tmpX + r * cos(i*Angle + Angle0), tmpY + r * sin(i*Angle + Angle0), tmpZ);
    }
    glEnd();
}

void IKSimple::drawOverlay()
{
    int screenX, screenY;
    mCamera.worldToScreen(mGoalPosition, screenX, screenY);

    glColor3f(1.0, 0.0, 1.0);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, mWindowWidth, 0, mWindowHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    DrawCircle(vec3(screenX, screenY, 0));
    glBegin(GL_LINES);
    glVertex3f(screenX + 10.0, screenY, 0);
    glVertex3f(screenX - 10.0, screenY, 0);
    glVertex3f(screenX, screenY + 10.0, 0);
    glVertex3f(screenX, screenY - 10.0, 0);
    glEnd();
    glPopMatrix();
}

void TW_CALL IKSimple::ResetIKCb(void* clientData)
{
    IKSimple* viewer = (IKSimple*)clientData;
    viewer->reset();
}

void IKSimple::loadIKJoints()
{
    for (int i = 0; i < (int) mEffectorData.size(); i++)
    {
        TwRemoveVar(mPoseEditBar, mEffectorData[i]->name.c_str());
        delete mEffectorData[i];
    }
    mEffectorData.clear();

    char buff[256];
    for (int i = 0; i < mActor.getNumTransforms(); i++)
    {
        Joint* joint = mActor.getByID(i);

        EffectorData* ld = new EffectorData;
        ld->viewer = this;
        ld->jointid = i;
        ld->name = joint->getName();
        mEffectorData.push_back(ld);

        sprintf(buff, " label='%s' group='Select Joint'", joint->getName().c_str());
        TwAddButton(mPoseEditBar, mEffectorData.back()->name.c_str(), SelectIKCb, ld, buff);
    }
}


void IKSimple::onMouseMotion(int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseMotionGLUT(pX, pY)) return;

    if (mModifierState == GLUT_ACTIVE_CTRL && mSelectedJoint > -1 && mButtonState == GLUT_LEFT_BUTTON)
    {
        //std::cout << pX << " " << pY << std::endl;
        if (mSelectedRecticle) // update pos with new p
        {
            double dsqr = DistanceSqr(vec3(mLastIKX, mWindowHeight - mLastIKY, 0), vec3(pX, pY, 0));
            //std::cout << dsqr << std::endl;
            if (dsqr > 300.0)
            {
                vec3 target(0, 0, 0);
                vec3 current(0, 0, 0);
                mCamera.screenToWorld(mLastIKX, mWindowHeight - mLastIKY, current);
                mCamera.screenToWorld(pX, mWindowHeight - pY, target);

                // Did we click on the IK sphere previously?
                vec3 eye = mCamera.getPosition();

                double distToScreen = Distance(eye, current);
                double distToScreenTarget = Distance(current, target);
                double distToObject = Distance(eye, mGoalPosition);
                double distToTarget = distToObject*distToScreenTarget / distToScreen; // similar triangles

                vec3 dir = target - current;
                dir.Normalize();
                dir = dir * distToTarget;
                mGoalPosition = mGoalPosition + dir;

                updateIK();
                mLastIKX = pX; 
                mLastIKY = pY;
            }
        }
    }


    int deltaX = mLastX - pX;
    int deltaY = mLastY - pY;
    bool moveLeftRight = abs(deltaX) > abs(deltaY);
    bool moveUpDown = !moveLeftRight;
    if (mModifierState != GLUT_ACTIVE_CTRL && mButtonState == GLUT_LEFT_BUTTON)  // Rotate
    {
        if (moveLeftRight && deltaX > 0) mCamera.orbitLeft(deltaX);
        else if (moveLeftRight && deltaX < 0) mCamera.orbitRight(-deltaX);
        else if (moveUpDown && deltaY > 0) mCamera.orbitUp(deltaY);
        else if (moveUpDown && deltaY < 0) mCamera.orbitDown(-deltaY);
    }
    else if (mButtonState == GLUT_MIDDLE_BUTTON) // Zoom
    {
        if (moveUpDown && deltaY > 0) mCamera.moveForward(deltaY);
        else if (moveUpDown && deltaY < 0) mCamera.moveBack(-deltaY);
    }
    else if (mButtonState == GLUT_RIGHT_BUTTON) // Pan
    {
        if (moveLeftRight && deltaX > 0) mCamera.moveLeft(deltaX);
        else if (moveLeftRight && deltaX < 0) mCamera.moveRight(-deltaX);
        else if (moveUpDown && deltaY > 0) mCamera.moveUp(deltaY);
        else if (moveUpDown && deltaY < 0) mCamera.moveDown(-deltaY);
    }

    mLastX = pX;
    mLastY = pY;
}

void IKSimple::onMouse(int pButton, int pState, int pX, int pY)
{
    // Are we in the recticle?
    ABasicViewer::onMouse(pButton, pState, pX, pY);

    if (mModifierState == GLUT_ACTIVE_CTRL && mSelectedJoint > -1)
    {
        int screenX, screenY;
        mCamera.worldToScreen(mGoalPosition, screenX, screenY);

        double d = (screenX - pX)*(screenX - pX) + (screenY - mWindowHeight + pY)*(screenY - mWindowHeight + pY);
        mSelectedRecticle = (d <= 100);  // recticle has radius 5 in screen space

        mLastIKX = pX;
        mLastIKY = pY;
    }
    else
    {
        mSelectedRecticle = false;
    }
}

void TW_CALL IKSimple::onSetGoalXCb(const void *value, void *clientData)
{
    IKSimple* viewer = ((IKSimple*)clientData);
    double v = *(const double *)value;  // for instance
    viewer->mGoalPosition[0] = v;
    viewer->updateIK();
}

void TW_CALL IKSimple::onGetGoalXCb(void *value, void *clientData)
{
    IKSimple* viewer = ((IKSimple*)clientData);
    *static_cast<double *>(value) = viewer->mGoalPosition[0];
}

void TW_CALL IKSimple::onSetGoalYCb(const void *value, void *clientData)
{
    IKSimple* viewer = ((IKSimple*)clientData);
    double v = *(const double *)value;  // for instance
    viewer->mGoalPosition[1] = v;
    viewer->updateIK();
}

void TW_CALL IKSimple::onGetGoalYCb(void *value, void *clientData)
{
    IKSimple* viewer = ((IKSimple*)clientData);
    *static_cast<double *>(value) = viewer->mGoalPosition[1];
}

void TW_CALL IKSimple::onSetGoalZCb(const void *value, void *clientData)
{
    IKSimple* viewer = ((IKSimple*)clientData);
    double v = *(const double *)value;  // for instance
    viewer->mGoalPosition[2] = v;
    viewer->updateIK();
}

void TW_CALL IKSimple::onGetGoalZCb(void *value, void *clientData)
{
    IKSimple* viewer = ((IKSimple*)clientData);
    *static_cast<double *>(value) = viewer->mGoalPosition[2];
}

void TW_CALL IKSimple::SelectIKCb(void* clientData)
{
}

