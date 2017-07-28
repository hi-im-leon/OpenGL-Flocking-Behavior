//------------------------------------------------------------------------
// Copyright (C) 2015 by Aline Normoyle

#ifndef Timer_H_
#define Timer_H_

#include <iostream>

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#endif

class ATimer
{
public:
    ATimer(void);
    ~ATimer(void);
    void reset();
    void start();
    void pause();
    void restart();
    double totalElapsedTime();

protected:
    double mTotalElapsedTime;
    bool mStarted;

#ifdef WIN32
    LARGE_INTEGER mStartTime;
    static LARGE_INTEGER gFrequency;
    static bool gInit;
#else
	 struct timeval mLastTime;
#endif
};

#endif
