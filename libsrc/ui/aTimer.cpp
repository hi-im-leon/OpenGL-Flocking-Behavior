//------------------------------------------------------------------------
// 2015 by alinen

#include "aTimer.h"

#ifdef WIN32
bool ATimer::gInit = false;
LARGE_INTEGER ATimer::gFrequency;
ATimer::ATimer()
{
    if (!gInit)
    {
        QueryPerformanceFrequency(&gFrequency);
        if (gFrequency.QuadPart == 0)
        {
            std::cout << "This computer does not support High Resolution Performance counter" << std::endl;
        }
        else
        {
            std::cout << "Timer frequency = " << double(gFrequency.QuadPart) << std::endl;
        }
        gInit = true;
    }   
    mTotalElapsedTime = 0.0;
    mStarted = false;
}


void ATimer::start()
{
    if (!mStarted) QueryPerformanceCounter(&mStartTime);
    mStarted = true;
}

void ATimer::pause()
{
    if (mStarted)
    {
        LARGE_INTEGER currentTime;
        QueryPerformanceCounter(&currentTime);
        mTotalElapsedTime += double(currentTime.QuadPart - mStartTime.QuadPart) / double(gFrequency.QuadPart);
        mStarted = false;
    }
}

double ATimer::totalElapsedTime()
{
    if (mStarted)
    {
        LARGE_INTEGER currentTime;
        QueryPerformanceCounter(&currentTime);
        mTotalElapsedTime += double(currentTime.QuadPart - mStartTime.QuadPart) / double(gFrequency.QuadPart);
        mStartTime = currentTime;
    }
    return mTotalElapsedTime;
}

#else
#include <iostream>
ATimer::ATimer()
{
    mTotalElapsedTime = 0.0;
    mStarted = false;
}

void ATimer::start()
{
    if (!mStarted) gettimeofday(&mLastTime, NULL);
    mStarted = true;
}

void ATimer::pause()
{
    if (mStarted)
    {
		  totalElapsedTime(); 
        mStarted = false;
    }
}

double ATimer::totalElapsedTime()
{
    if (mStarted)
    {
        struct timeval current;
        gettimeofday(&current, NULL);

        long seconds = current.tv_sec - mLastTime.tv_sec;
        long useconds = current.tv_usec - mLastTime.tv_usec;
        double mtime = ((seconds*1000 + useconds/1000.0)+0.5)/1000.0;

        mTotalElapsedTime += mtime;
        mLastTime = current;
    }
    return mTotalElapsedTime;
}
#endif

void ATimer::restart()
{
    reset();
    start();
}

ATimer::~ATimer()
{
}

void ATimer::reset()
{
    mTotalElapsedTime = 0.0;
    mStarted = false;
}
