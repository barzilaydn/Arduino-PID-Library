/********************************************************
 * Multiple PIDs with a PIDMaster
 * Running multiple PID instances in parallel with Timer interrupts for better sampling.
 ********************************************************/

#include <TimerOne.h>
#include <PID_v2.h>
#include <PIDMaster.h>

//PID variables
volatile double Right_S, Right_I, Right_O;
volatile double Left_S, Left_I, Left_O;
volatile double Top_S, Top_I, Top_O;

//PID for each rotation type.
PID RightPID(&Right_I, &Right_O, &Right_S, 1.0, 2.0, 3.0, false, 10);
PID LeftPID (&Left_I , &Left_O , &Left_S , 2.0, 3.0, 4.0, false, 10);
PID TopPID  (&Top_I  , &Top_O  , &Top_S  , 3.0, 4.0, 5.0, false, 10);
PIDMaster masterPID (&RightPID, &LeftPID, &TopPID);

void setup()
{
    //initialize the variables we're linked to
    Right_I = analogRead(0);
    Left_I = analogRead(1);
    Top_I = analogRead(2);
    
    Right_S = 45;
    Left_S = 15;
    Top_S = 0;
    
    RightPID.SetOutputLimits(0,255);
    LeftPID.SetOutputLimits(0,255);
    TopPID.SetOutputLimits(0,255);
    masterPID.Start();  //Starts the Timer which automatically handles all PID instances.
}

void loop()
{
    Right_I = analogRead(0);
    Left_I = analogRead(1);
    Top_I = analogRead(2);
    
    analogWrite(3, Right_O);
    analogWrite(4, Left_O);
    analogWrite(5, Top_O);
}


