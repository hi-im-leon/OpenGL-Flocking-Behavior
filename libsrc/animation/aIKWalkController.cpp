#include "aIKWalkController.h"
#include "GL/glut.h"
#include <algorithm>

#pragma warning (disable : 4018)

IKWalkController::IKWalkController()
{
}

IKWalkController::~IKWalkController()
{
}

void IKWalkController::loadReferenceMotion(const std::string& filename)
{
    mBVHController.load(filename);
    mSkeleton = mBVHController.getSkeleton(); // make copy
    mLeftLegController.setSkeleton(mSkeleton);
    mRightLegController.setSkeleton(mSkeleton);

    mLFoot = mSkeleton.getByName("Beta:LeftFoot");
    mRFoot = mSkeleton.getByName("Beta:RightFoot");

    startLPos = mLFoot->getGlobalTranslation();
    startRPos = mRFoot->getGlobalTranslation();

    mLCurve.clear();
    mRCurve.clear();
    mOrigLCurve.clear();
    mOrigRCurve.clear();

    mLCurve.setFramerate(mBVHController.getFramerate());
    mRCurve.setFramerate(mBVHController.getFramerate());
    mOrigLCurve.setFramerate(mBVHController.getFramerate());
    mOrigRCurve.setFramerate(mBVHController.getFramerate());

    mOrigRootCurve.clear();
    mOrigRootCurve = mBVHController.getRootCurve();
    
    mContacts.clear();

    int numKeys = mBVHController.getNumKeys();
    double duration = mBVHController.getRootCurve().getDuration();
    vec3 displacement = mBVHController.getRootCurve().getKey(numKeys-1) - mBVHController.getRootCurve().getKey(0);

    double stepMin = 100.0;
    for (int i = 0; i < mBVHController.getNumKeys(); i++)
    {
        double time = i / ((double)mBVHController.getFramerate());

        Actor pose = mBVHController.getKey(i);
        vec3 lpos = pose.getByID(mLFoot->getID())->getGlobalTranslation();
        mLCurve.appendKey(time, lpos);
        mOrigLCurve.appendKey(time, lpos);

        vec3 rpos = pose.getByID(mRFoot->getID())->getGlobalTranslation();
        mRCurve.appendKey(time, rpos);
        mOrigRCurve.appendKey(time, rpos);

        stepMin = std::min<double>(stepMin, lpos[1]);
        stepMin = std::min<double>(stepMin, rpos[1]);
    }
    mStepMin = stepMin;
    computeContacts(0.1, mStepMin+0.1);
}

void IKWalkController::computeContacts(double velThreshold, double heightThreshold)
{
    mContacts.clear();
    mContacts = std::vector<FootContacts>(mRCurve.getNumKeys());
    for (unsigned int j = 0; j < mRCurve.getNumKeys(); j++)
    {
        int nextJ = (j + 1) % mRCurve.getNumKeys();
        vec3 vel = mRCurve.getKey(nextJ) - mRCurve.getKey(j);
        vel /= ((double) mBVHController.getFramerate());

        float height = mLCurve.getKey(j)[1];

        if (vel.SqrLength() < velThreshold*velThreshold && height < heightThreshold)
        {
            std::cout << "Right foot speed = " << vel.Length() << " height = " << height << std::endl;
            mContacts[j].push_back(mRFoot->getID());

            Actor pose = mBVHController.getKey(j);
            std::vector<vec3> poly = getSupportPolygon(pose, j, axisY);
            mLContacts.push_back(poly);
        }
    }

    for (unsigned int j = 0; j < mLCurve.getNumKeys(); j++)
    {
        int nextJ = (j + 1) % mLCurve.getNumKeys();
        vec3 vel = mLCurve.getKey(nextJ) - mLCurve.getKey(j);
        vel /= ((double)mBVHController.getFramerate());

        float height = mLCurve.getKey(j)[1];

        if (vel.SqrLength() < velThreshold*velThreshold && height < heightThreshold)
        {
            std::cout << "Left foot speed = " << vel.Length() << " height = " << height << std::endl;
            mContacts[j].push_back(mLFoot->getID());

            Actor pose = mBVHController.getKey(j);
            std::vector<vec3> poly = getSupportPolygon(pose, j, axisY);
            mRContacts.push_back(poly);
        }
    }
}

const Actor& IKWalkController::getSkeleton() const
{
    return mSkeleton;
}

void IKWalkController::edit(double strideScale, double heightScale)
{
    // Do something arbitrary to test
    vec3 loffset = mOrigLCurve.getKey(0);
    vec3 roffset = mOrigRCurve.getKey(0);
    mLCurve.clear();
    mRCurve.clear();

    for (unsigned int j = 0; j < mBVHController.getNumKeys(); j++)
    {
        vec3 lpos;
        vec3 rpos;

        vec3 lopos = mOrigLCurve.getKey(j);
        vec3 ropos = mOrigRCurve.getKey(j);

        lpos[0] = (lopos[0] - loffset[0]) * strideScale + loffset[0];
        rpos[0] = (ropos[0] - roffset[0]) * strideScale + roffset[0];

        lpos[1] = (lopos[1] - mStepMin) * heightScale + mStepMin;
        rpos[1] = (ropos[1] - mStepMin) * heightScale + mStepMin;

        lpos[2] = (lopos[2] - loffset[2]) * strideScale + loffset[2];
        rpos[2] = (ropos[2] - roffset[2]) * strideScale + roffset[2];

        double time = j / ((double)mBVHController.getFramerate());
        vec3 pos = mOrigRootCurve.getKey(j);
        pos[0] *= strideScale;
        pos[2] *= strideScale;
        mBVHController.getRootCurve().editKey(j, pos);

        mLCurve.appendKey(time, lpos);
        mRCurve.appendKey(time, rpos);
    }
}

void IKWalkController::update(double dt)
{
    mBVHController.update(dt);
    mSkeleton = mBVHController.getSkeleton();

    // TODO: USe curve
    int key = (int) (dt * mBVHController.getFramerate());
    key = key % mBVHController.getNumKeys();

    mLeftLegController.solveIKAnalytic(mLFoot->getID(), mLCurve.getValue(dt));
    mRightLegController.solveIKAnalytic(mRFoot->getID(), mRCurve.getValue(dt));
}

std::vector<vec3> MakeConvex(const std::vector<vec3>& poly, const vec3& UP)
{
    if (poly.size() == 0) return poly;

    // Calculate convex hull of points in poly
    // 3-coins algorithm was developed independently by 
    // Graham and Sklansky in 1972, to find convex hulls.
    // http://cgm.cs.mcgill.ca/~beezer/cs507/3coins.html

    int lowAxis = 1;
    if (UP == vec3(0, 1, 0)) lowAxis = 2;
    else lowAxis = 1;

    // 1. Find a start point
    int startindex = 0;
    double lowest = poly[0][lowAxis];
    for (unsigned int i = 0; i < poly.size(); i++)
    {
        if (poly[i][lowAxis] < lowest)
        {
            lowest = poly[i][lowAxis];
            startindex = i;
        }
    }
    // 2. Sort points radially
    vec3 start = poly[startindex];
    std::map<double, int> vertices;
    for (unsigned int i = 0; i < poly.size(); i++)
    {
        vec3 xaxis(1, 0, 0);
        vec3 test = poly[i] - start;
        if (test.Length() == 0)
        {
            vertices[-1.1] = startindex;
            continue;
        }
        test.Normalize();
        double key = test * xaxis;
        vertices[key] = i;
    }

    // 3. Go through other points radially from our start point
    std::vector<vec3> newPoly;
    for (std::map<double, int>::iterator it = vertices.begin(); it != vertices.end(); ++it)
    {
        newPoly.push_back(poly[it->second]);
    }

    startindex = 0;
    int back = startindex;   // labels
    int center = (startindex + 1) % newPoly.size(); // labels
    int front = (startindex + 2) % newPoly.size();  // labels

    int numvertices = newPoly.size();
    for (int i = 0; i < numvertices; i++)
    {
        vec3 pback = newPoly[back];
        vec3 pcenter = newPoly[center];
        vec3 pfront = newPoly[front];

        vec3 a = pcenter - pback;
        vec3 b = pfront - pcenter;
        vec3 cross = a^b;
        if ((UP == axisZ && cross[2] <= 0) ||
            (UP == axisY && cross[1] >= 0))  // left turn, proceed 
        {
            back = (back + 1) % newPoly.size();   // labels
            center = (center + 1) % newPoly.size(); // labels
            front = (front + 1) % newPoly.size();  // labels            
        }
        else
        {
            std::vector<vec3>::iterator pit;
            pit = std::find(newPoly.begin(), newPoly.end(), pcenter);
            newPoly.erase(pit);

            front = (front + newPoly.size() - 1) % newPoly.size();
            center = (center + newPoly.size() - 1) % newPoly.size();
            back = (back + newPoly.size() - 1) % newPoly.size();
        }
    }
    return newPoly;
}

std::vector<vec3> IKWalkController::getSupportPolygon(Actor& pose, int keyID, const vec3& up)
{
    double eps = 5.0; // some padding
    std::vector<vec3> contactPoly;
    FootContacts contacts = mContacts[keyID];

    for (unsigned int i = 0; i < contacts.size(); i++)
    {
        Joint* joint = pose.getByID(contacts[i]);
        vec3 startpt = joint->getLocal2Global() * vec3(0, 0, 0);
        vec3 endpt = joint->getChildAt(0)->getLocal2Global() * vec3(0, 0, 0);

        int UPaxis = (up == axisY ? 1 : 2);
        startpt[UPaxis] = 0.0;
        endpt[UPaxis] = 0.0;

        vec3 UP = up;
        vec3 margin1 = eps*(startpt - endpt).Normalize();
        vec3 margin2 = (startpt - endpt) ^ UP;
        margin2 = eps*margin2.Normalize();

        contactPoly.push_back(startpt + margin1 + margin2);
        contactPoly.push_back(startpt + margin1 - margin2);
        contactPoly.push_back(endpt - margin1 - margin2);
        contactPoly.push_back(endpt - margin1 + margin2);
    }

    contactPoly = MakeConvex(contactPoly, vec3(0,1,0));
    return contactPoly;
}

void DrawPolygon(const std::vector<vec3>& poly, const vec3& color, const vec3& up)
{
    glColor3f(color[0], color[1], color[2]);

    vec3 offset = 0.05 * up;
    glPushMatrix();
    glTranslatef(offset[0], offset[1], offset[2]);
    glBegin(GL_LINE_LOOP);
    for (unsigned int i = 0; i < poly.size(); i++)
        glVertex3f(poly[i][0], poly[i][1], poly[i][2]);
    glEnd();
    glPopMatrix();
}


void IKWalkController::drawOpenGL(float time)
{
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glColor3f(0, 1, 0);
    for (int i = 1; i < mLCurve.getNumKeys(); i++)
    {
        vec3 lpos1 = mLCurve.getKey(i - 1);
        vec3 lpos2 = mLCurve.getKey(i);
        glVertex3f(lpos1[0], lpos1[1], lpos1[2]);
        glVertex3f(lpos2[0], lpos2[1], lpos2[2]);
    }
    glEnd();

    glBegin(GL_LINES);
    glColor3f(0, 0, 1);
    for (int i = 1; i < mRCurve.getNumKeys(); i++)
    {
        vec3 rpos1 = mRCurve.getKey(i - 1);
        vec3 rpos2 = mRCurve.getKey(i);
        glVertex3f(rpos1[0], rpos1[1], rpos1[2]);
        glVertex3f(rpos2[0], rpos2[1], rpos2[2]);
    }
    glEnd();
    glLineWidth(1.0);

    vec3 lpos = mLCurve.getValue(time);
    glPushMatrix();
    glTranslatef(lpos[0], lpos[1], lpos[2]);
    glutSolidSphere(1.0, 8, 8);
    glPopMatrix();

    /*
    int key = (int)(time * mBVHController.getFramerate());
    key = key % mBVHController.getNumKeys();
    Actor pose = mBVHController.getKey(key);
    std::vector<vec3> poly = getSupportPolygon(pose, key, axisY);
    DrawPolygon(poly, vec3(1, 0, 0), axisY);
    */
    
    for (int i = 0; i < mLContacts.size(); i++)
    {
        DrawPolygon(mLContacts[i], vec3(1, 0, 0), axisY);
    }

    for (int i = 0; i < mRContacts.size(); i++)
    {
        DrawPolygon(mRContacts[i], vec3(1, 0, 0), axisY);
    }
}
