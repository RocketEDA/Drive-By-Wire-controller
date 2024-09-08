#include <arduino-timer.h>
//#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <EEPROM.h>


////////////////////////////////////Throttle Body//////////////////////////////////////
#define PWM_FREQ 500    //tb pwm frequency

//PCA9685 pins
#define LPWM_pin 0      //PWM pin on PCA9685
#define RPWM_pin 1      //PWM pin on PCA9685
#define EN_pin 3        //R_EN/L_EN tied together
//#define RIS_pin         //overcurrent alarm right
//#define LIS_pin         //overcurrent alarm right

#define PROPGAIN 20 //proportional gain, motor force (12b) = (TB TPS(10b) - pedal pos (10b)) * PROPGAIN
                    //decrease if oscillating, increase for better throttle response and precision

#define MINPWM 1600     //minimum PWM for the motor to having action
#define PWMFREQ 500     //servo pwm frequency

//////////////////////////////////////TPS///////////////////

//TPS 1/2 maximum variation -- used for error checking if one redundant sensor has failed
#define TBTPS_var 200
#define PTPS_var 300

//throttle body pot pins
#define TBTPS1_pin A0
#define TBTPS2_pin A1

//e-pedal pot pins
#define PTPS1_pin A2
#define PTPS2_pin A3

//min throttle idle pot pin
#define IDLE_POT_pin A6

//maximum possible throttle plate open at idle (0-1023)
#define MAX_IDLE_TB_OPEN 200

//////////////////////////////////////Other
//programming macros
#define OPEN 1
#define CLOSE 0

#define INUM 100  //control loop interval in uS

#define EEPROM_ADDR 100   //address to store pedal TPS values
#define SETUP_pin 2       //setup switch, apply 5V to activate

//throttle body tps sensor
float tbtps1_val = 0;
float tbtps2_val = 0;

//pedal tps sensor
float ptps1_val = 0;
float ptps2_val = 0;

//throttle body TPS max/min values
float tbtps1_OPEN = 0;
float tbtps1_CLOSE = 0;
float tbtps1_range = 0;   //range = open-close. precalculate for faster runtime

float tbtps2_OPEN = 0;
float tbtps2_CLOSE = 0;
float tbtps2_range = 0;   //range = open-close. precalculate for faster runtime

//pedal TPS max/min values
float ptps1_OPEN = 0;
float ptps1_CLOSE = 0;
float ptps1_range = 0;   //range = open-close. precalculate for faster runtime

float ptps2_OPEN = 0;
float ptps2_CLOSE = 0;
float ptps2_range = 0;   //range = open-close. precalculate for faster runtime

float idle_pos = 0;   //minimum throttle position

float throttlePos = 0;   //throttle pedal position average reading

float mot_pwm = 0;  //motor force control follows writeMot convention

float tbAvg = 0; //throttle body position average reading


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Timer<3, micros> u_timer;

/*
   readTBTPS
   directly reads throttle body TPS
   raw value
*/
void readTBTPS()
{
  tbtps1_val = analogRead(TBTPS1_pin);
  tbtps2_val = analogRead(TBTPS2_pin);
}

/*
   readPTPS
   directly reads pedal TPS
   raw value
*/
void readPTPS()
{
  //invert if pins are inversed
  ptps1_val = analogRead(PTPS2_pin);
  ptps2_val = analogRead(PTPS1_pin);
}
/*
   calculateTBTPS
   abstracts readTBTPS
   calculates tbAvg
*/
bool calculateTBTPS()
{
  readTBTPS();
  //read and calculate current motor position
  float tbTemp1 = abs(((tbtps1_val - tbtps1_CLOSE) * 1024) / tbtps1_range);
  float tbTemp2 = abs(((tbtps2_CLOSE - tbtps2_val) * 1024) / tbtps2_range);
  //average between the two, use
  tbAvg = abs((tbTemp1 + tbTemp2) / 2);
  //Serial.println("tbtemp1: " + String(tbTemp1) + "\ttbtemp2: " +  String(tbTemp2));
  if (abs(tbTemp1 - tbTemp2) > TBTPS_var)
  {
    //error
    Serial.println("readTPS: ERROR INVALID TBTPS " + String(tbTemp1) + "\t" +  String(tbTemp2));
    //read and calculate current motor position
    
    while (1)
    {
      emergencyStop();
    }
    
  }
  return true;
}

/*
   calculateTPTPS
   abstracts readPTPS
   calculates throttlePos
*/
bool calculatePTPS()
{
  readPTPS();
  //pedal tps sensor
  //read and calculate current pedal position, based on range, pos close and pos open
  float pTemp1 = abs(((ptps1_val - ptps1_CLOSE) * 1023) / ptps1_range);
  float pTemp2 = abs(((ptps2_val - ptps2_CLOSE) * 1023) / ptps2_range);
  throttlePos = idle_pos + ((abs((pTemp1 + pTemp2) / 2.0) / 1023.0) * (1023 - idle_pos));    //average and scale with idle position
  throttlePos = constrain(throttlePos, 0, 1023);
  if (abs(pTemp1 - pTemp2) > PTPS_var)
  {
    //error
    Serial.println("readTPS: ERROR INVALID PTPS " + String(pTemp1) + "\t" +  String(pTemp2));
    //read and calculate current motor position
    
    while (1)
    {
      emergencyStop();
    }
    
  }
  //Serial.println(String(pTemp1) + "\t" + String(pTemp2) + "\t" + String(throttlePos));
  return true;
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
   val: 13 bit, 0-4095 is close, 4096 is relax, 4097-8192 is open
*/
void writeMot(float val)
{
  /*
     RPWM+ LPWM- to open
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
    //Serial.println("CLOSE: " + String(val));
  }
  else if (val > 4096)
  {
    //open
    digitalWrite(EN_pin, HIGH);
    pwm.setPWM(RPWM_pin, 0, val - 4097);
    pwm.setPWM(LPWM_pin, 0, 0);
    //Serial.println("OPEN: " + String(val));
  }
  else if (val == 4096)
  {
    //relax
    digitalWrite(EN_pin, LOW);
    pwm.setPWM(RPWM_pin, 0, 0);
    pwm.setPWM(LPWM_pin, 0, 0);
    //Serial.println("HOLD: " + String(val));
  }

}

/*
   abstracts writeMot
   sets position of throttle body based on global throttlePos
   interval controller so call on interval
*/
bool setPos()
{
  if (tbAvg < throttlePos)
  {
    mot_pwm = 4096 + (PROPGAIN * (throttlePos - tbAvg));
  }
  else if (tbAvg > throttlePos)
  {
    mot_pwm = 4096 + (PROPGAIN * (throttlePos - tbAvg));
  }
  mot_pwm = constrain(mot_pwm, 0, 8192);
  //Serial.println(String(mot_pwm) + "\t" + String(tbAvg) + "\t" + String(throttlePos));
  //Serial.println(String(throttlePos - tbAvg));
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
  writeMot(8191);
  delay(500);
  //while(1);
  //read TB TPS values
  readTBTPS();
  tbtps1_OPEN = tbtps1_val;
  tbtps2_OPEN = tbtps2_val;

  //close motor
  writeMot(0);
  delay(500);

  //read TB TPS values
  readTBTPS();
  tbtps1_CLOSE = tbtps1_val;
  tbtps2_CLOSE = tbtps2_val;

  //calculate ranges
  tbtps1_range = abs(tbtps1_OPEN - tbtps1_CLOSE);
  tbtps2_range = abs(tbtps2_OPEN - tbtps2_CLOSE);

  Serial.println("TPS1OPEN: " + String(tbtps1_OPEN) + " TPS1CLOSE: " + String(tbtps1_CLOSE) + " TPS2OPEN: " + String(tbtps2_OPEN) + " TPS2CLOSE: " + String(tbtps2_CLOSE));
  //Serial.println("tbtps1_range: " + String(tbtps1_range) + " tbtps2_range: " + String(tbtps2_range));
  writeMot(4096); //relax motor


  /*
     checks if setup switch (pin) is pulled HIGH
     allows setting calibration values if it is
     else reads values from eeprom
  */
  bool setup_en = digitalRead(SETUP_pin);
  Serial.println("Setup mode: " + String(setup_en));
  if (setup_en)
  {
    //enter setup
    Serial.println("Push throttle all the way then release. Do this a few times to capture the range");
    readPTPS();
    idle_pos = analogRead(IDLE_POT_pin);
    ptps1_OPEN = ptps1_val;
    ptps1_CLOSE = ptps1_val;
    ptps2_OPEN = ptps2_val;
    ptps2_CLOSE = ptps2_val;
    while (setup_en)
    {
      readPTPS();
      //find max/mins
      if (ptps1_val > ptps1_OPEN)
      {
        ptps1_OPEN = ptps1_val;
      }
      if (ptps1_val < ptps1_CLOSE)
      {
        ptps1_CLOSE = ptps1_val;
      }
      if (ptps2_val > ptps2_OPEN)
      {
        ptps2_OPEN = ptps2_val;
      }
      if (ptps2_val < ptps2_CLOSE)
      {
        ptps2_CLOSE = ptps2_val;
      }
      //set test idle position to throttlePos, write to motor
      throttlePos = (float(analogRead(IDLE_POT_pin)) * MAX_IDLE_TB_OPEN) / 1023;
      calculateTBTPS();
      setPos();
      setup_en = digitalRead(SETUP_pin);
    }
    writeMot(4096);  //relax motor
    //save to eeprom   EEPROM_ADDR
    Serial.println("Writing EEPROM...");
    EEPROM.put(EEPROM_ADDR, ptps1_OPEN);                            //pedal1 open
    EEPROM.put(EEPROM_ADDR + sizeof(float), ptps1_CLOSE);           //pedal1 close
    EEPROM.put(EEPROM_ADDR + (2 * sizeof(float)), ptps2_OPEN);      //pedal2 open
    EEPROM.put(EEPROM_ADDR + (3 * sizeof(float)), ptps2_CLOSE);     //pedal2 close
    EEPROM.put(EEPROM_ADDR + (4 * sizeof(float)), throttlePos);     //idle throttle position

    Serial.println("ptps1_OPEN: " + String(ptps1_OPEN));
    Serial.println("ptps1_CLOSE: " + String(ptps1_CLOSE));
    Serial.println("ptps2_OPEN: " + String(ptps2_OPEN));
    Serial.println("ptps2_CLOSE: " + String(ptps2_CLOSE));
    Serial.println("IDLE: " + String(throttlePos));
  }
  Serial.println("Reading EEPROM...");
  //not setup mode, read from eeprom
  EEPROM.get(EEPROM_ADDR, ptps1_OPEN);                            //pedal1 open
  EEPROM.get(EEPROM_ADDR + sizeof(float), ptps1_CLOSE);           //pedal1 close
  EEPROM.get(EEPROM_ADDR + (2 * sizeof(float)), ptps2_OPEN);      //pedal2 open
  EEPROM.get(EEPROM_ADDR + (3 * sizeof(float)), ptps2_CLOSE);     //pedal2 close
  EEPROM.get(EEPROM_ADDR + (4 * sizeof(float)), idle_pos);     //idle throttle position
  //calculate ranges
  ptps1_range = abs(ptps1_OPEN - ptps1_CLOSE);
  ptps2_range = abs(ptps2_OPEN - ptps2_CLOSE);

  Serial.println("ptps1_OPEN: " + String(ptps1_OPEN));
  Serial.println("ptps1_CLOSE: " + String(ptps1_CLOSE));
  Serial.println("ptps2_OPEN: " + String(ptps2_OPEN));
  Serial.println("ptps2_CLOSE: " + String(ptps2_CLOSE));
  Serial.println("ptps1_range: " + String(ptps1_range));
  Serial.println("ptps2_range: " + String(ptps2_range));
  Serial.println("IDLE: " + String(idle_pos));
}


void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(PWMFREQ);
  Wire.setClock(400000);

  pinMode(EN_pin, OUTPUT);
  pinMode(SETUP_pin, INPUT);
  initSequence();
  //u_timer.every(INUM, calculateTBTPS);
  //u_timer.every(INUM, calculatePTPS);
  //u_timer.every(INUM, setPos);
}
void loop() {
  delayMicroseconds(INUM);
  calculateTBTPS();
  calculatePTPS();
  setPos();
}
