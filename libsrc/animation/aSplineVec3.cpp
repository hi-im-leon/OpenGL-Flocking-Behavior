#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>

ASplineVec3::ASplineVec3() : 
    mInterpolator(new ALinearInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

    delete mInterpolator;
    switch (type)
    {
    case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
    case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
    case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
    case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
    case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < (int) mKeys.size());
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < (int) mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints();
    }
    else if (ID == (int) mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
        computeControlPoints();
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));


    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < (int) mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID)
{
    assert(keyID >= 0 && keyID < (int) mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return (int) mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID)
{
    assert(ID >= 0 && ID < (int) mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == (int) mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return (int) mCtrlPoints.size()+2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys.size() > 0? mKeys[mKeys.size()-1].first : 0;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

vec3 ASplineVec3::getValue(double t)
{
    if (mCachedCurve.size() == 0) return vec3();

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    int i = rawi % mKeys.size();
    double frac = (t - rawi*dt)/dt;
    int inext = i + 1;
    if (!mLooping) inext = std::min<int>(inext, mKeys.size() - 1);
    else inext = inext % mKeys.size();

    vec3 v1 = mKeys[i].second;
    vec3 v2 = mKeys[inext].second;
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints()
{
    if (mKeys.size() >= 2)
    {
        int totalPoints = mKeys.size();

        //If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
        //They lie on the tangent of the first and last interpolation points.
        vec3 tmp = mKeys[0].second - mKeys[1].second;
        double n = tmp.Length();
        mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

        tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
        n = tmp.Length();
        mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
    }

    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : 
    mType(t), mDt(1.0 / 120.0)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
    curve.clear();

    for (int segment = 1; segment < (int) keys.size(); segment++)
    {
        for (double t = keys[segment-1].first; t < keys[segment].first - EPSILON; t += mDt)
        {
            double u = (t-keys[segment-1].first)/(keys[segment].first-keys[segment-1].first);
            vec3 val = interpolateSegment(keys, ctrlPoints, segment-1, u);
            curve.push_back(val);
        }
    }
    if (keys.size() > 0) curve.push_back(keys[keys.size() - 1].second); // add last point
}


vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
    vec3 key1 = keys[segment].second;
    vec3 key2 = keys[segment+1].second;
    vec3 item = key1*(1.0-u)+key2*u;
    return item;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
    vec3 b0 = ctrlPoints[segment * 4];
    vec3 b1 = ctrlPoints[segment * 4 + 1];
    vec3 b2 = ctrlPoints[segment * 4 + 2];
    vec3 b3 = ctrlPoints[segment * 4 + 3];
    vec3 xyz;
    xyz = (pow((1-t), 3) * b0)+ (3 * pow((1-t),2) * t * b1) + (3 * pow(t, 2) * (1-t) * b2)+ (pow(t, 3) * b3);
    return xyz;
}

vec3 AInterpolatorVec3::lerp(const vec3 a, const vec3 b, double t) {
    return a * (1-t) + b * t;
}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
    vec3 b0 = ctrlPoints[segment * 4];
    vec3 b1 = ctrlPoints[segment * 4 + 1];
    vec3 b2 = ctrlPoints[segment * 4 + 2];
    vec3 b3 = ctrlPoints[segment * 4 + 3];
    vec3 b10 = lerp(b0, b1, t);
    vec3 b11 = lerp(b1, b2, t);
    vec3 b12 = lerp(b2, b3, t);
    vec3 b20 = lerp(b10, b11, t);
    vec3 b21 = lerp(b11, b12, t);
    vec3 xyz = lerp(b20, b21, t);
    return xyz;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
    Eigen::MatrixXd cntrlpnts = Eigen::MatrixXd::Zero(4, 3);

    vec3 b0 = ctrlPoints[segment * 4];
    vec3 b1 = ctrlPoints[segment * 4 + 1];
    vec3 b2 = ctrlPoints[segment * 4 + 2];
    vec3 b3 = ctrlPoints[segment * 4 + 3];
    
    cntrlpnts(0,0) = b0[0];
    cntrlpnts(0,1) = b0[1];
    cntrlpnts(0,2) = b0[2];

    cntrlpnts(1,0) = b1[0];
    cntrlpnts(1,1) = b1[1];
    cntrlpnts(1,2) = b1[2];
    
    cntrlpnts(2,0) = b2[0];
    cntrlpnts(2,1) = b2[1];
    cntrlpnts(2,2) = b2[2];
    
    cntrlpnts(3,0) = b3[0];
    cntrlpnts(3,1) = b3[1];
    cntrlpnts(3,2) = b3[2];

    Eigen::MatrixXd multmatrix = Eigen::MatrixXd::Zero(4,4);
    multmatrix << 1, 0, 0, 0,
                  -3, 3, 0, 0,
                  3, -6, 3, 0,
                  -1, 3, -3, 1;


    Eigen::MatrixXd curr_time = Eigen::MatrixXd::Zero(1, 4);
    curr_time(0,0) = 1.0;
    curr_time(0,1) = t;
    curr_time(0,2) = pow(t,2);
    curr_time(0,3) = pow(t,3);

    Eigen::MatrixXd res = curr_time * multmatrix * cntrlpnts;
    return vec3(res(0, 0), res(0, 1), res(0, 2) );
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
    vec3 p0 = keys[segment].second;
    vec3 p1 = keys[segment + 1].second;
    vec3 q0 = ctrlPoints[segment];
    vec3 q1 = ctrlPoints[segment + 1];

    vec3 xyz = p0 * (1-3 * pow(t, 2) + 2 * pow(t, 3)) + q0 * 
    (t - 2 * pow(t, 2) + pow(t, 3)) + q1 * (-pow(t, 2) + pow(t, 3)) +
    p1 * (3 * pow(t, 2) - 2 * pow(t, 3));

    return xyz;
}

void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;
    for (int i = 0; i < keys.size()-1; i++) {
        vec3 b0, b1, b2, b3;
        if (i == 0) {
            b0 = keys[0].second;
            b1 = b0 + (keys[0].second - startPoint);
            ctrlPoints.push_back(b0);
            ctrlPoints.push_back(b1);
        } else {
            b0 = keys[i].second;
            ctrlPoints.push_back(b0);
            b1 = b0 + (1.0/6.0)*(keys[i+1].second-keys[i-1].second);
            ctrlPoints.push_back(b1);
        }
        if (i == keys.size()-2) {
            b3 = keys[i+1].second;
            b2 = b3 - (endPoint - keys[i+1].second);
            ctrlPoints.push_back(b2);
            ctrlPoints.push_back(b3);
        } else {
            b3 = keys[i+1].second;
            b2 = b3 - (1.0/6.0)*(keys[i+2].second-keys[i].second);
            ctrlPoints.push_back(b2);
            ctrlPoints.push_back(b3);
        }
    }
}

void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();

    if (keys.size() <= 1) return;
    if (keys.size() == 2) {
        Eigen::MatrixXd multmatrix = Eigen::MatrixXd::Zero(2,2);
        multmatrix << 2, 1,
                      1, 2;
        vec3 p0 = keys[0].second;
        vec3 p1 = keys[1].second;
        vec3 first(3 * (p0-p1));
        Eigen::MatrixXd pointmatrix = Eigen::MatrixXd::Zero(2,3);
        pointmatrix << first[0], first[1], first[2],
                       first[0], first[1], first[2];
        Eigen::MatrixXd slopes = multmatrix.inverse() * pointmatrix;
        ctrlPoints.push_back(vec3(slopes(0,0), slopes(0,1), slopes(0,2)));
        ctrlPoints.push_back(vec3(slopes(1,0), slopes(1,1), slopes(1,2)));
        return;
    }

    Eigen::MatrixXd multmatrix = Eigen::MatrixXd::Zero(keys.size(), keys.size());
    // Set first row
    multmatrix(0,0) = 2;
    multmatrix(0,1) = 1;

    // Set last row
    multmatrix(keys.size() - 1, keys.size() - 2) = 1;
    multmatrix(keys.size() - 1, keys.size() - 1) = 2;

    for (int i = 1; i < keys.size() - 1; i++) {
        multmatrix(i,i-1) = 1;
        multmatrix(i,i) = 4;
        multmatrix(i,i+1) = 1;
    }

    Eigen::MatrixXd pointmatrix = Eigen::MatrixXd::Zero(keys.size(), 3);
    vec3 row1 = 3 * (keys[1].second - keys[0].second);
    vec3 rowlast = 3 * (keys[keys.size() - 1].second - keys[keys.size() - 2].second);
    pointmatrix(0,0) = row1[0];
    pointmatrix(0,1) = row1[1];
    pointmatrix(0,2) = row1[2];
    
    pointmatrix(keys.size() - 1,0) = rowlast[0];
    pointmatrix(keys.size() - 1,1) = rowlast[1];
    pointmatrix(keys.size() - 1,2) = rowlast[2];

    for (int i = 1; i < keys.size() - 1; i++) {
        vec3 row = 3 * (keys[i + 1].second - keys[i - 1].second);
        pointmatrix(i, 0) = row[0];
        pointmatrix(i, 1) = row[1];
        pointmatrix(i, 2) = row[2];
    }
    Eigen::MatrixXd resMatrix =  multmatrix.inverse() * pointmatrix;
    // std::cout << "MultMatrix: " << std::endl << multmatrix << std::endl << "Point matrix:" 
    // << std::endl << pointmatrix << std::endl << "Result: " << std::endl << resMatrix << std::endl;
    for (int i = 0; i < keys.size(); i++) {
        vec3 v(resMatrix(i, 0), resMatrix(i, 1), resMatrix(i, 2));
        ctrlPoints.push_back(v);   
    }
}

