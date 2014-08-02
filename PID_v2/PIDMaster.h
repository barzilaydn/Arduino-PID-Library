/**********************************************************************************************
* Arduino PIDMaster Library - Version 1.0
* v1.0 by Dan Barzilay <barzilaydn@gmail.com>
*
* This Library is licensed under a LGPLv3 License
**********************************************************************************************/
#ifndef PIDMaster_h
#define PIDMaster_h
#define LIBRARY_VERSION	1.0.0

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <TimerOne.h>
#include <PID_v2.h>
#include <vector.h>
#include <stdarg.h>

class PIDMaster
{
public:
    PIDMaster(PID*, ... ); //Constructor
    void Attach(PID*, ... );
    void Start();
    void Restart();
    void Stop();
    void Resume();
    void Compute();
    
    static PIDMaster* self;
    
private:
    unsigned long GCD(unsigned long u, unsigned long v);
    
    Vector<PID*> children;        //Pointer to use array of PID instances.
    volatile unsigned long gcdSampleTime;  //in microseconds.
    volatile unsigned long maxSampleTime;  //in microseconds.
    unsigned int count;
};

//PIDMaster* PIDMaster::self;
void computeWrapper();

#endif