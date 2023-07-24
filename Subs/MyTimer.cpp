//
// <MyTimer.cpp> See Header for details
//
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>		// malloc, system, random atof, atoi etc

#include "MyDataTypes.h"
#include "MyTimer.h"
#include "FMat.h"

MyTimer::MyTimer(int ID)
{
    m_Ident=ID;
    m_Tick=0;
    m_Tock=0;
}

void  MyTimer::StartTimer()
{
//
// resets m_Start
//
    gettimeofday(&m_Start, NULL);
}

MYDWORD MyTimer::MyMillis()
{
//
// This function replicates the functionallity of the Arduino millis()
// function which returns the elapsed time in milliseconds since the above
// StartTimer() function was called. This is slightly different to the Arduino
// implementation which starts from when the device is turned on, call behaviour 
// and usage is otherwise identical
//
	long int useconds, seconds;
	MYDWORD msec;  

	gettimeofday(&m_Now,NULL);

	seconds  = m_Now.tv_sec  - m_Start.tv_sec;
    useconds = m_Now.tv_usec - m_Start.tv_usec;

    msec = (MYDWORD)(useconds/1000 + seconds*1000);

	return msec;
}

void MyTimer::WriteStart()
{
    StartTimer();
    FILE* pTimeFile = fopen("TickTock.txt","w+");
    fprintf(pTimeFile," %ld, %ld \n",m_Start.tv_sec,m_Start.tv_usec);
}

int MyTimer::ReadStart()
{
    FString Line, Word;
    int Idx=0;
    FILE* pTimeFile = fopen("TickTock.txt","r");
    Idx=0;
    if(pTimeFile!=NULL){
        FindNextLine(pTimeFile,Line);

        FindNextWord(Line,Word,Idx);
        m_Start.tv_sec = atol(Word);

        FindNextWord(Line,Word,Idx);
        m_Start.tv_usec = atol(Word);
        return 1;
    } else {
        return 0;
    }
}

MYDWORD MyTimer::MyMicros()
{
//
// returns time elapsed since timer started in micro-seconds
// more care is needed since it will
	long int useconds, seconds;
	MYDWORD usec;  

	gettimeofday(&m_Now,NULL);

	seconds  = m_Now.tv_sec  - m_Start.tv_sec;
    useconds = m_Now.tv_usec - m_Start.tv_usec;

    usec = (MYDWORD)(useconds + seconds*1000000);

	return usec;
}

void MyTimer::Tick()
{
    m_Tick = MyMillis();
}

MYDWORD  MyTimer::Tock()
{
// returns the time delta since Tick() 
// or -1 if Tick() not previously set
// it will reset Tick to 0 to so can
// only be used in sequence with Tick()
//
    MYDWORD delta;

    if(m_Tick>0){
        m_Tock = MyMillis();
        delta = (int)(m_Tock-m_Tick);
        return delta;
    } else {
        return -1;
    }
}

void MyTimer::MuTick()
{
    m_Tick = MyMicros();
}

MYDWORD  MyTimer::MuTock()
{
// returns the time delta since Tick() 
// or -1 if Tick() not previously set
// it will reset Tick to 0 to so can
// only be used in sequence with Tick()
//
    MYDWORD delta;

    if(m_Tick>0){
        m_Tock = MyMicros();
        delta = m_Tock-m_Tick;
        return delta;
    } else {
        return 0;
    }
}
