#include "aSplineQuaternion.h"
#include <algorithm>

ASplineQuaternion::ASplineQuaternion() : 
    mDt(1.0 / 120.0), mDirty(false), mLooping(true), mType(LINEAR)
{
}

ASplineQuaternion::~ASplineQuaternion()
{
}

void ASplineQuaternion::setInterpolationType(ASplineQuaternion::InterpolationType type)
{
    mType = type;
    cacheCurve();
}

ASplineQuaternion::InterpolationType ASplineQuaternion::getInterpolationType() const
{
    return mType;
}

void ASplineQuaternion::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineQuaternion::getLooping() const
{
    return mLooping;
}

void ASplineQuaternion::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double ASplineQuaternion::getFramerate() const
{
    return 1.0 / mDt;
}

Quaternion ASplineQuaternion::getValue(double time)
{
	 if (mDirty) cacheCurve();
    if (mKeys.size() == 0) return Quaternion();

    int rawi = (int)(time / mDt);
    int i = rawi % mKeys.size();
    int inext = (i + 1) % mKeys.size();
    if (!mLooping) inext = std::min<int>(i+1, mKeys.size() - 1);
    Quaternion key1 = mKeys[i].second;
    Quaternion key2 = mKeys[inext].second;
    double frac = (time - rawi*mDt)/mDt;
    return Quaternion::Slerp(key1, key2, frac);
}

void ASplineQuaternion::cacheCurve() const
{
    if (mType == LINEAR || mKeys.size() < 3) interpolateLinear();
    else interpolateCubic();
	 mDirty = false;
}

void ASplineQuaternion::interpolateLinear() const
{
    mCachedCurve.clear();
    for (int i = 1; i < (int) mKeys.size(); i++)
    {
        for (double t = mKeys[i - 1].first; t < mKeys[i].first - EPSILON; t += mDt)
        {
            double frac = (t - mKeys[i-1].first) / (mKeys[i].first - mKeys[i-1].first);
            Quaternion key1 = mKeys[i-1].second;
            Quaternion key2 = mKeys[i].second;
            Quaternion q = Quaternion::Slerp(key1, key2, frac);
            mCachedCurve.push_back(q);
        }
    }
    if (mKeys.size() > 0) mCachedCurve.push_back(mKeys[mKeys.size() - 1].second); // add last point
}

void ASplineQuaternion::interpolateCubic() const
{
    mCachedCurve.clear();

    int intervalA = 0;
    int intervalStart = 0;
    int intervalEnd = 1;
    int intervalB = 2;
    while (intervalEnd < (int) mKeys.size() && intervalStart != intervalEnd)
    {
        double intervalLength = (double)(mKeys[intervalEnd].first - mKeys[intervalStart].first);
        assert(intervalLength > 0.000001);

        Quaternion A = mKeys[intervalA].second;
        Quaternion start = mKeys[intervalStart].second;
        Quaternion end = mKeys[intervalEnd].second;
        Quaternion B = mKeys[intervalB].second;

        for (double t = mKeys[intervalStart].first; t < mKeys[intervalEnd].first; t += mDt)
        {
            double frac = (t - mKeys[intervalStart].first) / intervalLength;
            Quaternion s0 = Quaternion::Intermediate(A, start, end);
            Quaternion s1 = Quaternion::Intermediate(start, end, B);
            Quaternion q = Quaternion::Squad(start, s0, s1, end, frac);
            mCachedCurve.push_back(q);
        }
        intervalA = intervalStart;
        intervalStart = intervalEnd;
        intervalEnd = intervalB;
        intervalB = intervalB + 1;
        if (intervalB+1 > (int) mKeys.size())
        {
            intervalB = intervalEnd;
        }
    }
}

void ASplineQuaternion::editKey(int keyID, const Quaternion& value)
{
    assert(keyID >= 0 && keyID < (int) mKeys.size());
    mKeys[keyID].second = value;
	 mDirty = true;
}

void ASplineQuaternion::appendKey(const Quaternion& value, bool updateCurve)
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

void ASplineQuaternion::appendKey(double t, const Quaternion& value, bool updateCurve)
{
    mKeys.push_back(Key(t, value));
	 mDirty = true;
}

void ASplineQuaternion::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < (int) mKeys.size());
    mKeys.erase(mKeys.begin() + keyID - 1);
	 mDirty = true;
}

Quaternion ASplineQuaternion::getKey(int keyID)
{
    assert(keyID >= 0 && keyID < (int) mKeys.size());
    return mKeys[keyID].second;
}

int ASplineQuaternion::getNumKeys() const
{
    return mKeys.size();
}

void ASplineQuaternion::clear()
{
    mKeys.clear();
}

double ASplineQuaternion::getDuration() const
{
	 if (mDirty) cacheCurve();
    return mCachedCurve.size() * mDt;
}

double ASplineQuaternion::getNormalizedTime(double t) const
{
    double duration = getDuration();
    int rawi = (int)(t / duration);
    return t - rawi*duration;
}

