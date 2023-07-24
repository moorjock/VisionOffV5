//
// PorcessState.cpp
//
// displays process state and changes to process sched_ options and affinity
//
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>   // ERROR Number Definitions
#include <string.h>  //  memset etc
#include <poll.h>    // keyboard polling 
#include <sched.h>   // schedulling

#include "MyDataTypes.h"
#include "FMat.h"
#include "MyUnixSerial.h"
#include "CommTypes.h"
#include "ProcessState.h"


void print_process_state(pid_t pid, int Procs)
{
    int schedType, cpu, prio, niceval;
	struct sched_param schParam; 
    cpu_set_t cpuset;
//
    if(pid==0){
        pid = getpid();
    }

    schedType = sched_getscheduler(pid);

    switch(schedType)
    {
        case SCHED_DEADLINE:
	          PrintM(" Schedule  Policy is SCHED_DEADLINE");
            break;
        case SCHED_FIFO:
	          PrintM(" Schedule Policy is SCHED_FIFO");
	        break;
        case SCHED_RR:
	          PrintM(" Schedule  Policy is SCHED_RR");
	        break;
        case SCHED_OTHER:
	          PrintM(" Schedule  Policy is SCHED_OTHER");
            break;
        default:
            PrintM(" Schedule  Policy is UNKNOWN");
    }
    sched_getparam(pid, &schParam);
    prio = schParam.sched_priority;
//
    niceval = nice(0);
// get affinity
    CPU_ZERO(&cpuset);
    sched_getaffinity(pid, sizeof(cpu_set_t), &cpuset);

    for(int i=0;i<Procs; i++){
        cpu = CPU_ISSET(i,&cpuset);
        if(cpu>0){
            cpu = i;
            break;
        }
    }

// output priority and affinity and frequency for this pid
    sprintf(ErrorBuf," Schedule Priority %d CPU Affinity %d nice %d, for pid %d\n",prio, cpu,niceval,(int)pid);
    PrintM(ErrorBuf);

}

void set_schedule(pid_t pid, int Policy, int Priority)
{
	struct sched_param schParam; 

    if(pid<=0){
        pid = getpid();
    }

// Set the priority to a middle value

	schParam.sched_priority = Priority;
//
// set the schedule priority
//
	if (sched_setscheduler(pid, Policy, &schParam) <0){
        perror("sched_setscheduler");
    }
}

int set_nice(int increment)
{
//
// For normal priority (CFS) processes nice value can be used to
// increase or reduce the processes time slice it can also
// affect a processes IO priority in the CFQ
//
// Valid nice values are -20 <= nice <= +19 the default is nice=0
// with -ve values increasing CFS amd CFQ process priority
// values of nice<0 can only be sey by super users
//
    int newNice;

    errno=0;
    newNice = nice(increment);
    if( (newNice = -1) && (errno != 0) ){
        perror("nice");
    }
    return newNice;
}

void Set_CPU_Affinity(pid_t pid, int cpuIdx)
{
//
// set CPU affinity
//
    cpu_set_t cpuset;
    int rc=0;

    if(pid==0){
        pid = getpid();
    }

    if(cpuIdx>-1){
        CPU_ZERO(&cpuset);
        CPU_SET(cpuIdx, &cpuset);

        rc = sched_setaffinity(pid,sizeof(cpu_set_t), &cpuset);

        if(rc<0){
            perror("sched_setaffinity");
        }
    } else {
        PrintM(" CPU Affinity not set");
    }
}
