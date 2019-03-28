#include "Arduino.h"
#include "Motors.h"

Motor::Motor(uint8_t MPWMpin, uint8_t FPIN, uint8_t RPIN, uint8_t pin1, uint8_t pin2)
    :MPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT), MEncoder(pin1,pin2)
{
  PWMpin=-1;
  IN1pin=-1;
  IN2pin=-1;
  prevCount=0;
  Setpoint=0; 
  MPID.SetMode(AUTOMATIC);
  MPID.SetOutputLimits(-255,255);
  PWMpin = MPWMpin;
  IN1pin = FPIN;
  IN2pin = RPIN;
  pinMode(PWMpin, OUTPUT);
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);
  
}

void Motor::run(uint8_t cmd){
  if(cmd == FORWARD){
    digitalWrite(IN1pin,LOW);
    digitalWrite(IN2pin,HIGH);
  }
  else if(cmd == BACKWARD){
    digitalWrite(IN2pin,LOW);
    digitalWrite(IN1pin,HIGH);    
  }  
  else if(cmd == STOP){
    digitalWrite(IN1pin,LOW);
    digitalWrite(IN2pin,LOW);
  }  
}

void Motor::setSpeed(uint8_t pwm){
  analogWrite(PWMpin, pwm);
}

void Motor::updatePID() {
  Position = MEncoder.read();
  Input = getRPM(abs(Position));

  MPID.Compute();

  MPWM = constrain(int(MPWM+Output),0,255);
  if (Setpoint==0) 
    MPWM=0;

  setSpeed(MPWM);
}

double Motor::getRPM(long count) {
  double speed_act = ((count - prevCount)*(60*(1000/LOOPTIME)))/(2700.0);
  prevCount = count;
  return abs(speed_act);
}

long Motor::getPosition() {
  Position = MEncoder.read();
  return Position;
}

void Motor::resetPosition(){
  Position = 0;
  MEncoder.write(0);
}

void Motor::changeTunings(double p, double i, double d){
  MPID.SetTunings(p,i,d);
}



