/**********************************************************************************************
* Arduino PIDMaster Library - Version 1.0
* v1.0 by Dan Barzilay <barzilaydn@gmail.com>
*
* This Library is licensed under a LGPLv3 License
**********************************************************************************************/
#include <PIDMaster.h>

PIDMaster* PIDMaster::self;

/* Constructor (int k, PID&, ...)******************************************
*  Load the children, Sets up the timer.
***************************************************************************/
PIDMaster::PIDMaster(PID* pid, ... )
{
    self = this;
    PID* p;
    unsigned long t;

    va_list argptr;               // A place to store the list of arguments
    va_start ( argptr, pid );     // Initializing arguments to store all values after first.
    
    unsigned long gcdTempTime = (*pid).GetSampleTime() * 1000; //temp to avoid multiple readings of volatile.
    unsigned long maxTempTime = gcdTempTime; //temp to avoid multiple readings of volatile.

    //Re-compute minSampleTime
    for(p = pid; p != NULL; p = va_arg(argptr, PID*))
    {
        t = (unsigned long)((*p).AttachToMaster()) * 1000; //milliseconds to microseconds.
        
        gcdTempTime = PIDMaster::GCD(t, gcdTempTime);
        
        if(t > maxTempTime)
            maxTempTime = t;
        
        children.push_back(p);   //Add new PIDs
    }
    
    Timer1.initialize(gcdTempTime); //In microseconds.
    Timer1.stop();
    
    gcdSampleTime = gcdTempTime;
    maxSampleTime = maxTempTime;
    
    va_end ( argptr );             // Cleans up the list
}

void PIDMaster::Attach(PID* pid, ... )
{    
    PIDMaster::Stop();

    PID* p;
    unsigned long t;
    
    va_list argptr;                // A place to store the list of arguments
    va_start ( argptr, pid );      // Initializing arguments to store all values after first.
    
    unsigned long gcdTempTime = gcdSampleTime; //temp to avoid multiple readings of volatile.
    unsigned long maxTempTime = maxSampleTime; //temp to avoid multiple readings of volatile.

    //Re-compute minSampleTime
    for(p = pid; p != NULL; p = va_arg(argptr, PID*))
    {
        t = (unsigned long)((*p).AttachToMaster()) * 1000; //milliseconds to microseconds.
        
        gcdTempTime = GCD(t, gcdTempTime);
        
        if(t > maxTempTime)
            maxTempTime = t;
        
        children.push_back(p);   //Add new PIDs
    }
    
    Timer1.setPeriod(gcdTempTime); //In microseconds.
    Timer1.attachInterrupt(computeWrapper); // Compute to run every GCD sample time.
    
    gcdSampleTime = gcdTempTime;
    maxSampleTime = maxTempTime;

    va_end ( argptr );             // Cleans up the list
    
    PIDMaster::Resume();    //Resumes
}

unsigned long PIDMaster::GCD(unsigned long u, unsigned long v)
{
    return (v != 0) ? PIDMaster::GCD(v, u%v) : u;
}

void computeWrapper()
{
    (*(PIDMaster::self)).Compute();
}

void PIDMaster::Compute()
{
    PIDMaster::Stop();
    
    PID* p;
    unsigned long sample = gcdSampleTime;
    
    for (size_t i = 0; i < children.size(); ++i)
    {
        p = children[i];
        if(sample * count % ((*p).GetSampleTime() * 1000) == 0 && ((*p).GetMode() != 0) && (*p).GetMasterAttached())
            (*p).Compute(); //Compute only when PID child is active and sample time arrived.
    }
    
    if(count < (unsigned int)(maxSampleTime / sample))
        count++;
    else
        count = 0;
        
    PIDMaster::Resume();
    
    return;
}

void PIDMaster::Start()
{
    if(children.size() != 0)
    {
        count = 0; //Restart count
        Timer1.attachInterrupt(computeWrapper);
        Timer1.start();
    }
    
    return;
}

void PIDMaster::Restart()
{
    if(children.size() != 0)
    {
        count = 0; //Restart count
        Timer1.attachInterrupt(computeWrapper);
        Timer1.restart();
    }
    
    return;
}

void PIDMaster::Stop()
{
    Timer1.stop();
    Timer1.detachInterrupt();
    
    return;
}

void PIDMaster::Resume()
{
    if(children.size() != 0)
    {
        Timer1.attachInterrupt(computeWrapper);
        Timer1.resume();
    }
    
    return;
}