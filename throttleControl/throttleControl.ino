#include <arduino-timer.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>



////////////////////////////////////Throttle Body//////////////////////////////////////
#define PWM_FREQ 500    //tb pwm frequency

//PCA9685 pins
#define LPWM_pin 6      //PWM pin on PCA9685
#define RPWM_pin 7      //PWM pin on PCA9685
#define EN_pin 4        //R_EN/L_EN tied together
//#define RIS_pin         //overcurrent alarm right
//#define LIS_pin         //overcurrent alarm right

#define CLOSE_VAL 127 //hold close force (0-255), adds to springback

#define PROPGAIN 32 //proportional gain, motor force (12b) = (TB TPS(10b) - pedal pos (10b)) * PROPGAIN
#define MINPWM 1600     //minimum PWM for the motor to having action
#define PWMFREQ 500     //servo pwm frequency

//////////////////////////////////////Throttle Position sensor - include TB TPS///////////////////

//TPS 1/2 maximum variation -- used for error checking if one redundant sensor has failed
#define TBTPS_var 100
#define PTPS_var 100

//throttle body pot pins
#define TBTPS1_pin A0
#define TBTPS2_pin A1

//e-pedal pot pins
#define PTPS1_pin A2
#define PTPS2_pin A3

//////////////////////////////////////Other
//programming macros
#define OPEN 1
#define CLOSE 0

#define INUM 100  //PID loop interval in uS




//throttle body tps sensor
long tbtps1_val = 0;
long tbtps2_val = 0;

//pedal tps sensor
long ptps1_val = 0;
long ptps2_val = 0;

//throttle body TPS max/min values
long tbtps1_OPEN = 0;
long tbtps1_CLOSE = 0;
long tbtps1_range = 0;   //range = open-close. precalculate for faster runtime

long tbtps2_OPEN = 0;
long tbtps2_CLOSE = 0;
long tbtps2_range = 0;   //range = open-close. precalculate for faster runtime

//pedal TPS max/min values
long ptps1_OPEN = 0;
long ptps1_CLOSE = 0;
long ptps1_range = 0;   //range = open-close. precalculate for faster runtime

long ptps2_OPEN = 0;
long ptps2_CLOSE = 0;
long ptps2_range = 0;   //range = open-close. precalculate for faster runtime

//throttle position
double throttlePos = 0;

double mot_pwm = 0;  //motor force control, 9 bit, 0-127 is close, 128-255 is open

double tbAvg = 0; //throttle body average reading


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Timer<1, micros> u_timer;

/*
   readTPS
   reads TPS values into global variable
*/
void readTPS()
{
  //read and calculate current motor position
  long tbTemp1 = abs(((analogRead(TBTPS1_pin) - tbtps1_CLOSE) * 1024) / tbtps1_range);
  long tbTemp2 = abs(((tbtps2_CLOSE - analogRead(TBTPS2_pin)) * 1024) / tbtps2_range);
  //average between the two, use
  tbAvg = abs((tbTemp1 + tbTemp2) / 2);

  if (abs(tbTemp1 - tbTemp2) > TBTPS_var)
  {
    //error
    while (1)
    {
      tbTemp1 = abs(((analogRead(TBTPS1_pin) - tbtps1_CLOSE) * 255) / tbtps1_range);
      tbTemp2 = abs(((tbtps2_CLOSE - analogRead(TBTPS2_pin)) * 255) / tbtps2_range);
      //average between the two, use
      tbAvg = abs((tbTemp1 + tbTemp2) / 2);
      Serial.println("readTPS: ERROR INVALID TPS " + String(tbTemp1) + "\t" +  String(tbTemp2) + "\t" + String(tbtps1_val) + "\t" + String(tbtps2_val) + "\t" + String(tbAvg) + "\t" + String(throttlePos));
      //read and calculate current motor position
      emergencyStop();
    }
  }

  //pedal tps sensor
  //read and calculate current pedal position, based on range, pos close and pos open
  long pTemp1 = abs(((analogRead(PTPS1_pin) - ptps1_CLOSE) * 1024) / ptps1_range);
  long pTemp2 = abs(((analogRead(PTPS2_pin) - ptps2_CLOSE) * 1024) / ptps2_range);
  //average between the two, use
  throttlePos = abs((pTemp1 + pTemp2) / 2);

  //Serial.println(String(pTemp1) + "\t" + String(pTemp2) + "\t" + String(throttlePos));


}
/*
   emergencyStop
   call this when in error state
   concentrates the numbers in one area, decreases chance for error
*/
void emergencyStop()
{
  digitalWrite(EN_pin, LOW);
}
/*
   writeMot
   writes to motor
   val: 13 bit, 0-4095 is close, 4096 is hold, 4097-8192 is open
*/
void writeMot(long val)
{
  /*
   * RPWM+ LPWM- to open
   */
  if (val > 8192 || val < 0)
  {
    Serial.println("writeMot: ERROR VAL EXCEEDED " + String(val));
    emergencyStop();
    while (1);
  }
  else if (val < 4096)
  {
    //close
    digitalWrite(EN_pin, HIGH);
    pwm.setPWM(RPWM_pin, 0, 0);
    pwm.setPWM(LPWM_pin, 0, 4095 - val);
    Serial.println("CLOSE: " + String(val));
  }
  else if (val > 4096)
  {
    //open
    digitalWrite(EN_pin, HIGH);
    pwm.setPWM(RPWM_pin, 0, val - 4097);
    pwm.setPWM(LPWM_pin, 0, 0);
    Serial.println("OPEN: " + String(val));
  }
  else if (val == 4096)
  {
    //hold
    digitalWrite(EN_pin, HIGH);
    pwm.setPWM(RPWM_pin, 0, 0);
    pwm.setPWM(LPWM_pin, 0, 0);
    Serial.println("HOLD: " + String(val));
  }

}

/*
   abstracts writeMot
   sets position of throttle body based on global throttlePos
   interval controller so call on interval
*/
bool setPos()
{
  readTPS();
  if (tbAvg < throttlePos)
  {
    if (throttlePos > 15)
    {
      mot_pwm = 4096 + (PROPGAIN * (throttlePos - tbAvg));
    }
    else
    {
      mot_pwm = 4096;  //off throttle noise, just filter it out
    }
  }
  else if (tbAvg > throttlePos)
  {
    mot_pwm = 4096 + (PROPGAIN * (throttlePos - tbAvg));
  }
  mot_pwm = constrain(mot_pwm, 0, 8192);
  Serial.println(String(mot_pwm) + "\t" + String(tbAvg) + "\t" + String(throttlePos));
  writeMot(mot_pwm);
  return true;
}
/*
   read max/min TB values
*/
void initSequence()
{
  Serial.println("initialization sequence");
  //////////////////////////////TB setup
  //open motor
  writeMot(8192);
  delay(500);

  //read TB TPS values
  readTPS();
  tbtps1_OPEN = analogRead(TBTPS1_pin);
  tbtps2_OPEN = analogRead(TBTPS2_pin);

  //close motor
  writeMot(0);
  delay(500);

  //read TB TPS values
  readTPS();
  tbtps1_CLOSE = analogRead(TBTPS1_pin);
  tbtps2_CLOSE = analogRead(TBTPS2_pin);

  tbtps1_range = abs(tbtps1_OPEN - tbtps1_CLOSE);
  tbtps2_range = abs(tbtps2_OPEN - tbtps2_CLOSE);

  Serial.println("TPS1OPEN: " + String(tbtps1_OPEN) + " TPS1CLOSE: " + String(tbtps1_CLOSE) + " TPS2OPEN: " + String(tbtps2_OPEN) + " TPS2CLOSE: " + String(tbtps2_CLOSE));
  Serial.println("tbtps1_range: " + String(tbtps1_range) + " tbtps2_range: " + String(tbtps2_range));
  writeMot(4096); //relax motor

  //////////////////EPEDAL SETUP
  Serial.println("Setting ePedal min. Release throttle, press s to continue");
  char a = 'a';
  while (a != 's')
  {
    a = Serial.read();
  }
  ptps1_CLOSE = analogRead(PTPS1_pin);
  ptps2_CLOSE = analogRead(PTPS2_pin);

  Serial.println("Setting ePedal max. Depress throttle all the way, press s to continue");
  a = 'a';
  while (a != 's')
  {
    a = Serial.read();
  }
  ptps1_OPEN = analogRead(PTPS1_pin);
  ptps2_OPEN = analogRead(PTPS2_pin);

  ptps1_range = ptps1_OPEN - ptps1_CLOSE;
  ptps2_range = ptps2_OPEN - ptps2_CLOSE;

  Serial.println("ptps1_OPEN: " + String(tbtps1_OPEN) + " ptps1_CLOSE: " + String(ptps1_CLOSE) + " ptps2_OPEN: " + String(ptps2_OPEN) + " ptps2_CLOSE: " + String(ptps2_CLOSE));
  Serial.println("ptps1_range: " + String(ptps1_range) + " ptps2_range: " + String(ptps2_range));
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(PWMFREQ);
  Wire.setClock(400000);

  pinMode(EN_pin, OUTPUT);
  initSequence();

  u_timer.every(INUM, setPos);
}
void loop() {
  setPos();
}
