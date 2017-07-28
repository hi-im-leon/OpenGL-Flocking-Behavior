#include "aCharacterDrawer.h"
#include "aVector.h"
#include <GL/glut.h>
#include <algorithm>
#include <iostream>

using namespace std;

const GLfloat SPECULAR[] = {0.3f, 0.3f, 0.3f, 1.0f};
const vec3 LIGHT_BLUE(0.459f, 0.729f, 1.0f);

const vec3 DARK_BLUE(0.0510, 0.1255, 0.4627);
const vec3 DARK_YELLOW(0.9922, 0.8392, 0.1725);
const vec3 LIGHT_RED(1.0, 0.5, 0.5);
const vec3 LIGHT_GREEN(0.0, 1.0, 0.25);

const float JOINT_RADIUS = 5.0f; 
const float COORD_LENGTH = 0.4f;

inline void DrawSphere(GLfloat radius)
{
    static GLUquadricObj *quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    /* If we ever changed/used the texture or orientation state
    of quadObj, we'd need to change it to the defaults here
    with gluQuadricTexture and/or gluQuadricOrientation. */
    gluSphere(quadObj, radius, 12, 12);
}

inline void DrawCylinder(GLfloat height, GLfloat radius)
{
    static GLUquadricObj *quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    /* If we ever changed/used the texture or orientation state
    of quadObj, we'd need to change it to the defaults here
    with gluQuadricTexture and/or gluQuadricOrientation. */
    gluCylinder(quadObj, radius, radius, height, 12, 12);
}

inline void DrawCone(GLfloat height, GLfloat radius)
{
    static GLUquadricObj *quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    /* If we ever changed/used the texture or orientation state
    of quadObj, we'd need to change it to the defaults here
    with gluQuadricTexture and/or gluQuadricOrientation. */
    gluCylinder(quadObj, 0, radius, height, 12, 12);
}

bool CharacterDrawer::gDraw = true;

CharacterDrawer::CharacterDrawer() 
{
}

CharacterDrawer::~CharacterDrawer()
{
}

void CharacterDrawer::draw(const Actor& skeleton)
{
    if (!gDraw) return;
    
    setMaterial(chosenColor);

    glPushMatrix();
    if (skeleton.getNumTransforms() > 0)
    {
        AnimatableTransform* pJoint = skeleton.getRoot();
        drawJoint(pJoint->getLocal2Global());

        int totalChildren = pJoint->getNumChildren();
        for (int i = 0; i < totalChildren; i++)
            drawBodyParts(pJoint->getChildAt(i));
    }
    glPopMatrix();
}

void CharacterDrawer::drawBodyParts(AnimatableTransform* currentJoint)
{
    drawJoint(currentJoint->getLocal2Global());

    AnimatableTransform* pParent = currentJoint->getParent();
    drawSphereLimb(pParent->getGlobalTranslation(), 
        currentJoint->getGlobalTranslation());

    // if (currentJoint->getName() == "Head") {
    //     cout << "WE FOUND THE HED" << endl;

    // }

    if (currentJoint->getName().find("Hand") != string::npos) return; 
    // don't draw hands
   
    int totalChildren = currentJoint->getNumChildren();
    for (int i = 0; i < totalChildren; i++)
        drawBodyParts(currentJoint->getChildAt(i));
}

void CharacterDrawer::setMaterial(const vec3& colorVec)
{
    GLfloat c[4];
    c[0] = colorVec[0];
    c[1] = colorVec[1];
    c[2] = colorVec[2];
    c[3] = 1.0;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0f);
}


void CharacterDrawer::glApplyRotation(const mat3& rotation)
{
    GLfloat mat[16];
    mat[0] = rotation[0][0]; mat[4] = rotation[0][1]; mat[8] = rotation[0][2];    mat[12] = 0.0f;
    mat[1] = rotation[1][0]; mat[5] = rotation[1][1]; mat[9] = rotation[1][2];    mat[13] = 0.0f;
    mat[2] = rotation[2][0]; mat[6] = rotation[2][1]; mat[10] = rotation[2][2];    mat[14] = 0.0f;
    mat[3] = 0.0f; mat[7] = 0.0f; mat[11] = 0.0f; mat[15] = 1.0f;
    glMultMatrixf(mat);
}

void CharacterDrawer::drawJoint(const Transform& globalTransform)
{
    const vec3& globalPosition = globalTransform.m_translation;
    const mat3& globalRotation = globalTransform.m_rotation;

    glPushMatrix();
        glTranslatef(globalPosition[0], globalPosition[1], globalPosition[2]);    
        glutSolidSphere(JOINT_RADIUS, 24, 12);

        if (false) // Draw axes
        {
            glApplyRotation(globalRotation);
            glDisable(GL_LIGHTING);
            glBegin(GL_LINES);
            glColor3f(1.0f,0.0f,0.0f);
            glVertex3f(0.0f,0.0f,0.0f);
            glVertex3f(COORD_LENGTH,0.0f,0.0f);

            glColor3f(0.0f,1.0f,0.0f);
            glVertex3f(0.0f,0.0f,0.0f);
            glVertex3f(0.0f,COORD_LENGTH,0.0f);

            glColor3f(0.0f,0.0f,1.0f);
            glVertex3f(0.0f,0.0f,0.0f);
            glVertex3f(0.0f,0.0f,COORD_LENGTH);
            glEnd();
            glEnable(GL_LIGHTING);
        }
    glPopMatrix();
}

void CharacterDrawer::drawSphereLimb(const vec3& startPosition, const vec3& endPosition)
{
    // Determine rotation axis
    vec3 direction = endPosition - startPosition; 
    float length = direction.Length();
    direction /= length;
    vec3 direction2 = Prod(direction, direction);
    float dx = direction2[1] + direction2[2];
    float dy = direction2[0] + direction2[2];
    float dz = direction2[0] + direction2[1];
    unsigned int eAxis;
    vec3 axis;
    float angle;
    if (dx >= dy && dx >= dz)
    {
        eAxis = 0;
        axis = axisX.Cross(direction);
        angle = acos(direction * axisX) * Rad2Deg;
    }
    else if (dy >= dx && dy >= dz)
    {
        eAxis = 1;
        axis = axisY.Cross(direction);
        angle = acos(direction * axisY) * Rad2Deg;
    }
    else
    {
        eAxis = 2;
        axis = axisZ.Cross(direction);
        angle = acos(direction * axisZ) * Rad2Deg;
    }
    glPushMatrix();
        vec3 center = (endPosition + startPosition) / 2.0f;
        vec3 scale;
        switch(eAxis)
        {
        case 0: scale.set(length / 2.0f, JOINT_RADIUS, JOINT_RADIUS); break;
        case 1: scale.set(JOINT_RADIUS, length / 2.0f, JOINT_RADIUS); break;
        case 2: scale.set(JOINT_RADIUS, JOINT_RADIUS, length / 2.0f); break;
        }
        glTranslatef(center[0], center[1], center[2]);
        glRotatef(angle, axis[0], axis[1], axis[2]);
        glScalef(scale[0], scale[1], scale[2]);
        glutSolidSphere (1.0, 24, 12);
    glPopMatrix();
}
