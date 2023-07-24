//
// ProcessState.h
//
#ifndef PROCESS_STATE_H
#define PROCESS_STATE_H

void print_process_state(pid_t pid, int Procs);

void set_schedule(pid_t pid, int Policy=SCHED_OTHER, int Priority=0);

int set_nice(int increment=0);

void Set_CPU_Affinity(pid_t pid=0, int cpuIdx=-1);

#define VERY_NICE 19
#define NICE 0
#define NOT_NICE -10
#define NASTY -20

#endif