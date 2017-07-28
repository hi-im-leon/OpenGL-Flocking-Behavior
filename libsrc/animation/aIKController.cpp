#include "aIKController.h"
#include "GL/glut.h"
#include <algorithm>
#include <math.h>
using namespace std;

int IKController::gIKMaxIters = 50;
double IKController::gIKEpsilon = 0.15;
int COUNT = 0;

IKController::IKController() : mSkeleton(0)
{

}

IKController::~IKController()
{
}

void IKController::drawOpenGL()
{
    glColor4f(0, 0, 1, 1);
    glBegin(GL_LINES);
    for (int i = 0; i < mSkeleton->getNumTransforms(); i++)
    {
        AnimatableTransform* joint = mSkeleton->getByID(i);
        Joint* pParent = joint->getParent();
        if (!pParent) continue;

        vec3 startPosition = pParent->getGlobalTranslation();
        vec3 endPosition = joint->getGlobalTranslation();

        glVertex3f(startPosition[0], startPosition[1], startPosition[2]);
        glVertex3f(endPosition[0], endPosition[1], endPosition[2]);
    }
    glEnd();
}

bool IKController::solveIKAnalytic(int jointid, const vec3& goalPos)
{
    // Implement analytic IK based on 3 joints 
    // The global position of joint should match the goalPos as closely as feasible
    Joint* joint = mSkeleton->getByID(jointid);
    Joint* middleJoint = joint->getParent();
    Joint* firstJoint = middleJoint->getParent();

    // Set Parent Joint's Rotation using Analytic
    double length2 = middleJoint->getLocalTranslation().Length();
    double length3 = joint->getLocalTranslation().Length();


    double r = (goalPos - firstJoint->getGlobalTranslation()).Length();
    double cosinephi = (r*r - length2*length2 -length3*length3)/(-2*length2*length3);
    if (cosinephi < -1) {
        cosinephi = -1;
    } else if (cosinephi > 1) {
        cosinephi = 1;
    }
    double phi = acos(cosinephi);
    vec3 limbDir = middleJoint->getLocalTranslation();
    limbDir.Normalize();
    vec3 axis = limbDir.Cross(vec3(0,0,-1)); 
    if (limbDir[1] < 0) axis = limbDir.Cross(vec3(0,0,1));
    double theta_2z = phi - M_PI;
    mat3 angle2;
    angle2.FromAxisAngle(axis, theta_2z);
    middleJoint->setLocalRotation(angle2);

    // cout << "Lengths: \n";
    // cout << "-------------------" << "\n" << "Middle Joint: " << length2 << 
    // " Grandparent: " << length3 << "\n";
    // cout << "Setting parent: \n";
    // cout << "r: " << r << " phi: " << phi << " theta_2z: " << theta_2z << "\n\n";

    // Set Grandparent Joint's Rotation using CCD
    vec3 error, direction_vec, cross;
    error = goalPos - joint->getGlobalTranslation();
    direction_vec = joint->getGlobalTranslation() - 
        firstJoint->getGlobalTranslation();
    cross = direction_vec.Cross(error); 
    if (cross.Length() == 0) {
        if (abs(1-Dot(direction_vec, error)) > 
                IKController::gIKEpsilon)  {
            // cout << "Collinear!" << "\n";
            return false;
        }
    }
    phi = atan2(cross.Length(), Dot(direction_vec, direction_vec) + 
        Dot(direction_vec, error));
    mat3 rotation;
    vec3 cross_prod = (firstJoint->getParent()->getLocal2Global()).Inverse().m_rotation * cross;
    rotation.FromAxisAngle(cross_prod/cross_prod.Length(), phi);          
    firstJoint->setLocalRotation(rotation * firstJoint->getLocalRotation());
    mSkeleton->updateTransforms();
    return true;
}

void IKController::setSelectedJoint(int selectedJoint, int chainSize)
{
    mSelectedJoint = selectedJoint;
    mIKChain.clear();
    if (mSelectedJoint >= 0 && mSelectedJoint < (int)mSkeleton->getNumTransforms())
    {
        Joint* tmp = mSkeleton->getByID(selectedJoint);
        if (tmp != NULL && tmp->getParent() != NULL)
        {
            // terminate at the joint before root joint, so that root will not change during IK	
            tmp = tmp->getParent();
            while (tmp->getParent() != NULL)
            {
                mIKChain.push_back(tmp);
                if (chainSize > -1 && (int) mIKChain.size() > chainSize) break;
                tmp = tmp->getParent();
            }
        }
    }
}

bool IKController::solveIKCCD(int jointid, const vec3& goalPos, int chainSize)
{
    if (mSelectedJoint != jointid || chainSize != mChainSize)
    {
        mChainSize = chainSize;
        setSelectedJoint(jointid, chainSize);
    }

    // There are no joints in the IK chain for manipulation
    if (mIKChain.size() == 0) return true;

    else {
        // Setting up vector(p)
        Joint* endEffector = mSkeleton->getByID(jointid);
        const vec3& jointPos = endEffector->getGlobalTranslation();
        vec3 error = goalPos - jointPos;
        double phi;
        vec3 cross;
        // cout <<"Joint Position: " << jointPos << endl;
        // cout <<"Goal Position: " << goalPos << endl;
        // cout <<"Error: " << error << endl;
        // cout << "Length of error x joint.localTranslation): " << errorCross.Length() << endl; 
        // cout << "------------------------" << endl << endl;
        
        // if (errorCross.Length() == 0) {
        //     return true;
        // }


        int iterations = 0;
        while ((error.Length() > IKController::gIKEpsilon) && (iterations < IKController::gIKMaxIters)) {
            for (int i = 0; i < mIKChain.size(); i++) {
                // cout << "LOOP ITERATION: " << i << endl;
                Joint * curJoint = mIKChain[i];
                error = goalPos - endEffector->getGlobalTranslation();
                const vec3& direction_vec = endEffector->getGlobalTranslation() - 
                    curJoint->getGlobalTranslation();
                cross = direction_vec.Cross(error); 

                if (cross.Length() == 0) {
                    if (abs(1-Dot(direction_vec, error)) > 
                        IKController::gIKEpsilon)  continue;
                }
                
                phi = 0.2 * atan2(cross.Length(), Dot(direction_vec, direction_vec) + 
                    Dot(direction_vec, error));

                mat3 rotation;
                // Cross is a direction matrix
                vec3 normalizedCross = (curJoint->getParent()->getLocal2Global()).Inverse().m_rotation * cross;
                rotation.FromAxisAngle(normalizedCross/normalizedCross.Length(), phi);          

                curJoint->setLocalRotation(rotation * curJoint->getLocalRotation());
                curJoint->updateTransformation();
                
            }
            iterations += 1;
        }
    }

    return false;
}
