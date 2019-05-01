#ifndef Motors_h
#define Motors_h

#include "Arduino.h"
#include "PID_v1.h"
#include "Encoder.h"

#define FORWARD    1
#define BACKWARD   2
#define STOP       3

#define LOOPTIME    100
#define CIRCUMFERENCE 6.28318530718

//GOOD VALUES
// #define Kp          0.07
// #define Ki          0.09
// #define Kd          0.024

//new PID good vals graph wise, less so drive wise
// #define Kp          0.07
// #define Ki          0.9
// #define Kd          0.09

// Kp 0.9
// Ki 0.02
// Kd 0.04

#define Kp          0.4
#define Ki          0.0005
#define Kd          0.001


class Motor
{
 public:
  Motor(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t);
  void run(uint8_t);
  void setSpeed(uint8_t);
  long getPosition();
  void resetPosition();
  Encoder MEncoder;
  PID MPID;
  void updatePID();
  double  Setpoint;
  double getRPM(long);  
  void changeTunings(double,double,double);
  double  Input, Output;

  private:
  uint8_t PWMpin, IN1pin, IN2pin, MPWM;  
  long Position, prevCount; 
};

#endif
