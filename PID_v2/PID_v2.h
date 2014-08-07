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
#ifndef PID_v2_h
#define PID_v2_h
#define LIBRARY_VERSION	2.0.0

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class PID
{
public:

    //commonly used functions **************************************************************************
    PID(double*, double*, double*,          // * constructor.  links the PID to the Input, Output, and 
    double, double, double, int, int);      //   Setpoint.  Initial tuning parameters are also set here

    PID(volatile double*, volatile double*, volatile double*,   // * constructor.  links the PID to the Input, Output, and 
    double, double, double, int, int);                          //   Setpoint.  Initial tuning parameters are also set here
       
   
    void SetMode(int Mode);                 // * 1 turns ON the PID and 0 turns it off.

    bool Compute();                // * performs the PID calculation.  it should be
                                            //   called every time loop() cycles when not in Interrupt mode. ON/OFF and
                                            //   calculation frequency can be set using SetMode
                                            //   SetSampleTime respectively

    void SetOutputLimits(double, double);//clamps the output to a specific range. 0-255 by default, but
    //it's likely the user will want to change this depending on
    //the application
    


    //available but not commonly used functions ********************************************************
    void SetTunings(double, double,  // * While most users will set the tunings once in the 
    double);                         //   constructor, this function gives the user the option
                                     //   of changing tunings during runtime for Adaptive control
    void ReverseControllerDirection(int);// * Sets the Direction, or "Action" of the controller. DIRECT
                                     //   means the output will increase when error is positive. REVERSE
                                     //   means the opposite.  it's very unlikely that this will be needed
                                     //   once it is set in the constructor.
  
    int AttachToMaster();
    
    //Display functions ****************************************************************
    double GetKp();               // These functions query the pid for interal values.
    double GetKi();               //  they were created mainly for the pid front-end,
    double GetKd();               // where it's important to know what is actually 
    int GetMode() volatile;       //  inside the PID.
    volatile unsigned long GetSampleTime() volatile;
    bool GetMasterAttached();
    bool GetDirection() volatile;
    
    // Input, Output, Setpoint getters and setters;
    volatile double GetInput() volatile;
    volatile double GetOutput() volatile;
    volatile double GetSetpoint() volatile;
    void SetInput(volatile double) volatile;
    void SetOutput(volatile double) volatile;
    void SetSetpoint(volatile double) volatile;
    bool SetDirection(volatile double) volatile;  
    
private:
    void Initialize();
    void SetSampleTime(int);    // * sets the frequency, in Milliseconds, with which 
                                //   the PID calculation is performed.  default is 100
    
    double dispKp;              // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;              //   format for display purposes
    double dispKd;              //
    
    volatile double kp;                  // * (P)roportional Tuning Parameter
    volatile double ki;                  // * (I)ntegral Tuning Parameter
    volatile double kd;                  // * (D)erivative Tuning Parameter

    bool controllerDirection;

    volatile double *myInput;   // * Pointers to the Input, Output, and Setpoint variables
    volatile double *myOutput;  //   This creates a hard link between the variables and the 
    volatile double *mySetpoint;//   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.
    
    unsigned long lastTime;
    volatile double ITerm, lastInput;
    volatile unsigned long sampleTime;
    bool masterAttached;
    
    volatile double outMin, outMax;
    volatile bool inAuto;
};
#endif

