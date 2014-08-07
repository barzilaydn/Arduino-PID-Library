/**********************************************************************************************
* Arduino PID Library - Version 2.0
* v1.0 by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
* v2.0 by Dan Barzilay <barzilaydn@gmail.com>

* v2.0 Changes:
*   - Added support for PIDMaster
*   - Improved Anti integral wind-up.
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/
#include <PID_v2.h>

#define intDisable()      ({ uint8_t sreg = SREG; cli(); sreg; })
#define intRestore(sreg)  SREG = sreg

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up 
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
double Kp, double Ki, double Kd, int ControllerDirection, int msSampleTime = 100)
{
    PID::SetSampleTime(msSampleTime);             //default is 100ms (0.1 seconds)
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    
    PID::SetOutputLimits(0, 255);				//default output limit corresponds to the arduino pwm limits

    PID::ReverseControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - msSampleTime;				
}

PID::PID(volatile double* Input, volatile double* Output, volatile double* Setpoint,
double Kp, double Ki, double Kd, int ControllerDirection, int msSampleTime = 100)
{
    PID::SetSampleTime(msSampleTime);             //default is 100ms (0.1 seconds)
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    
    PID::SetOutputLimits(0, 255);				//default output limit corresponds to the arduino pwm limits

    PID::ReverseControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - msSampleTime;				
}

/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/ 
bool PID::Compute()
{
    if(!inAuto) return false;

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    
    if(masterAttached || (!masterAttached && (timeChange >= sampleTime)))
    {
        /*Compute all the working error variables*/
        double input = *myInput;
        double error = *mySetpoint - input;
        ITerm += (ki * error);

        double dInput = (input - lastInput);

        /*Compute PID Output*/
        double output = kp * error + ITerm- kd * dInput;		
        
        /* Anti integral wind-up and output limiting*/
        if(output > outMax)
        {
            ITerm -= (output - outMax);
            output = outMax;
        }
        else if(output < outMin)
        {
            ITerm += (outMin - output);
            output = outMin;
        }

        *myOutput = output;

        /*Remember some variables for next time*/
        lastInput = input;        
        lastTime = now;
        
        return true;
    }
    return false;
}

/* AttachToMaster() **********************************************************************
*  Activates interrupt mode instead of polling by connecting this instance to a PIDMaster.
*
*  Returns the current sampleTime for the master to use.
**********************************************************************************/ 
int PID::AttachToMaster()
{
    masterAttached = true;
    return sampleTime;
}

/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted. 
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
    uint8_t sreg = intDisable();

    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    dispKp = Kp; dispKi = Ki; dispKd = Kd;

    double sampleTimeInSec = ((double)sampleTime)/1000;  
    kp = Kp;
    ki = Ki * sampleTimeInSec;
    kd = Kd / sampleTimeInSec;

    if(controllerDirection)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
    
    intRestore(sreg);
}

/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed	
******************************************************************************/
void PID::SetSampleTime(int NewsampleTime)
{
    uint8_t sreg = intDisable();

    if(inAuto) return;
    
    if (NewsampleTime > 0)
    {
        double ratio  = (double)NewsampleTime / (double)sampleTime;
        ki *= ratio;
        kd /= ratio;
        sampleTime = (unsigned long)NewsampleTime;
    }
    
    intRestore(sreg);
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
    uint8_t sreg = intDisable();
    
    if(Min >= Max) return;
    
    outMin = Min;
    outMax = Max;

    if(inAuto)
    {
        /* Anti integral wind-up and output limiting*/
        if(*myOutput > outMax)
        {
            ITerm -= (*myOutput - outMax);
            *myOutput = outMax;
        }
        else if(*myOutput < outMin)
        {
            ITerm += (outMin - *myOutput);
            *myOutput = outMin;
        }
    }
    
    intRestore(sreg);
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/ 
void PID::SetMode(int Mode)
{
    uint8_t sreg = intDisable();
    
    if(Mode && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = Mode;
    
    intRestore(sreg);
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/ 
void PID::Initialize()
{
    ITerm = *myOutput;
    lastInput = *myInput;
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;
}

/* ReverseControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads 
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
void PID::ReverseControllerDirection(int Direction)
{
    uint8_t sreg = intDisable();
    
    if(inAuto && Direction != controllerDirection)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }   
    controllerDirection = Direction;
    
    intRestore(sreg);
}

/* Status Functions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display 
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double                 PID::GetKp()                     { return dispKp;              }
double                 PID::GetKi()                     { return dispKi;              }
double                 PID::GetKd()                     { return dispKd;              }
int                    PID::GetMode() volatile          { return inAuto;              }
bool                   PID::GetDirection() volatile     { return controllerDirection; }
bool                   PID::GetMasterAttached()         { return masterAttached;      }
volatile unsigned long PID::GetSampleTime() volatile    { return sampleTime;          }

/* *myInput Setter & Getter */
void PID::SetInput(volatile double i) volatile
{ 
    uint8_t sreg = intDisable();
    *myInput = i;
    intRestore(sreg);
}

volatile double PID::GetInput() volatile
{
    uint8_t sreg = intDisable();
    volatile double i = *myInput;
    intRestore(sreg);
    return i;
}

/* *myOutput Setter & Getter */
void PID::SetOutput(double o) volatile
{ 
    uint8_t sreg = intDisable();
    *myOutput = o;
    intRestore(sreg);
}

volatile double PID::GetOutput() volatile
{
    uint8_t sreg = intDisable();
    volatile double o = *myOutput;
    intRestore(sreg);
    return o;
}

/* *mySetpoint Setter & Getter */
void PID::SetSetpoint(double s) volatile
{ 
    uint8_t sreg = intDisable();
    *mySetpoint = s;
    intRestore(sreg);
}

volatile double PID::GetSetpoint() volatile
{
    uint8_t sreg = intDisable();
    volatile double s = *mySetpoint;
    intRestore(sreg);
    return s;
}