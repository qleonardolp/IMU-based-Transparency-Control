//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#ifndef QPCLOOPTIMER_H
#define QPCLOOPTIMER_H

#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

// implementacao para linux... talvez usar std::chrono

// usado para manter precisao com long long int
#define MILLION 1000000

typedef long long int llint;

class looptimer
{
public:
    // Class Constructor
    looptimer(float sample_period, int duration) : lpInSeconds(sample_period), execDuration(duration){
#ifdef _WIN32
        lpInMicroseconds = MILLION*lpInSeconds;
        QueryPerformanceFrequency(&qpcFrequency);
        loopPeriodMSinTicks = qpcFrequency.QuadPart * lpInMicroseconds;
#endif

#ifdef linux
        loopPeriodMSinTicks = CLOCKS_PER_SEC*lpInSeconds;
#endif
    execDuration *= MILLION;
    }
    // Execution beginning timestamp
    void start(){
        execBegin = micro_now();
    }

    // Execution end check
    bool end(){
        return  (micro_now() - execBegin) > execDuration;
    }

    // Clock-based time (now) in microseconds
    long long int micro_now(){
#ifdef _WIN32
        QueryPerformanceCounter(&timeCounter);
        timeCounter.QuadPart *= MILLION;
        timeCounter.QuadPart /= qpcFrequency.QuadPart;
        return timeCounter.QuadPart;
#endif

#ifdef linux
        return int(MILLION*clock()/CLOCKS_PER_SEC);
#endif
    }

    // loop period start
    void tik(){
#ifdef _WIN32
        QueryPerformanceCounter(&tikCounter);
        beginning_t = MILLION*tikCounter.QuadPart;
        end_t = beginning_t + loopPeriodMSinTicks;
#endif

#ifdef linux
        beginning_t = clock();
        end_t = beginning_t + loopPeriodMSinTicks;
#endif
    }

    // loop period wait until lpInMicroseconds is reached
    void tak(){
#ifdef _WIN32
        do{
            QueryPerformanceCounter(&takCounter); 
        } while (end_t >= MILLION*takCounter.QuadPart);
#endif

#ifdef linux
        do{     } while (end_t >= clock());
#endif
    }

private:
    // lp = loop period
    float lpInSeconds;
    int execDuration;
    long long int execBegin;
    long long int lpInMicroseconds;
    long long int loopPeriodMSinTicks;
    long long int beginning_t;
    long long int end_t;
#ifdef _WIN32
    LARGE_INTEGER qpcFrequency;
    LARGE_INTEGER tikCounter;
    LARGE_INTEGER takCounter;
    LARGE_INTEGER timeCounter;
#endif
};

#endif //QPCLOOPTIMER_H

//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\////\//