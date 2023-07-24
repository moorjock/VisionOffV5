//
// <ScreenOut.cpp> See Header for Details
//
//
#include <sys/types.h> 
#include <sys/stat.h>

#include <fcntl.h>
#include <termios.h> 
#include <unistd.h>
#include <errno.h>


#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "FMat.h"
#include "ScreenOut.h"

char ScreenBuf[20];

ScreenOutPipe::ScreenOutPipe()
{
    m_FD=0;
    m_pipeName=NULL;
}

int ScreenOutPipe::Open(const char* pPipeName)
{
    m_pipeName = pPipeName;
    mkfifo(m_pipeName, 0666);
    m_FD = open(m_pipeName,O_WRONLY);
    return m_FD;
}

void ScreenOutPipe::Close()
{
    close(m_FD);
}

int ScreenOutPipe::ScreenOut(const char* pText)
{
// Text Output, note any formating of the text should be
// performed before this is called and the string NULL terminated
// otherwise a fail will occour
    
    int len = strlen(pText);
    return  write(m_FD,pText,len);;
}

int ScreenOutPipe::ScreenOut(int* pInts)
{
// output is standard csv format same as for results files
// line termination is not performed to allow concatenation
// from different sources on the same line
//
    int i, len = sizeof(pInts);
    for(i=0;i<len;i++){
        sprintf(ScreenBuf," %d,",pInts[i]);
        write(m_FD,ScreenBuf,strlen(ScreenBuf));
    }
    return len;
}

int ScreenOutPipe::ScreenOut(float* pFloat)
{
    int i, len = sizeof(pFloat);
    for(i=0;i<len;i++){
        sprintf(ScreenBuf,"  %#12.5E,",pFloat[i]);
        write(m_FD,ScreenBuf,strlen(ScreenBuf));
    }
    return len;
}

int ScreenOutPipe::ScreenOut(FVec& FV )
{
    int i, len;
    len = FV.Size();
    float* pData = FV.GetDataPointer();
    for(i=0;i<len;i++){
        ScreenOut(pData);
        pData+=sizeof(float);
    }
    return len;
}

int ScreenOutPipe::ScreenOut(double* pDoub)
{
    int i, len = sizeof(pDoub);

    for(i=0;i<len;i++){
        sprintf(ScreenBuf,"  %#12.5E,",pDoub[i]);
        write(m_FD,ScreenBuf,strlen(ScreenBuf));
    }
    return len;
}

void ScreenOutPipe::EndLn()
{
    write(m_FD,"\n",1);
}