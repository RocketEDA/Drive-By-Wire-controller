#include <arduino-timer.h>0

//l298n pins
#define EN_pin 3
#define IN1_pin 4
#define IN2_pin 7

#define CLOSE_VAL 127 //hold close force (0-255), adds to springback

//programming macros
#define OPEN 1
#define CLOSE 0

//TPS1/2 variation max
#define TBTPS_var 100
#define PTPS_var 100

//throttle body pot pins
#define TBTPS1_pin A0
#define TBTPS2_pin A1

//e-pedal pot pins
#define PTPS1_pin A2
#define PTPS2_pin A3

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
double throttlePos = 120;

double mot_pwm = 0;  //motor force control, 9 bit, 0-127 is close, 128-255 is open

double tbAvg = 0; //throttle body average reading




Timer<1, micros> u_timer;

/*
   readTPS
   reads TPS values into global variable
*/
void readTPS()
{
  //read and calculate current motor position
  long tbTemp1 = abs(((analogRead(TBTPS1_pin) - tbtps1_CLOSE) * 255) / tbtps1_range);
  long tbTemp2 = abs(((tbtps2_CLOSE - analogRead(TBTPS2_pin)) * 255) / tbtps2_range);
  //average between the two, use
  tbAvg = abs((tbTemp1 + tbTemp2) / 2);

  if (abs(tbTemp1 - tbTemp2) > TBTPS_var)
  {
    //error
    while (1)
    {
      Serial.println("setPos: ERROR INVALID TPS " + String(tbTemp1) + "\t" +  String(tbTemp2) + "\t" + String(tbtps1_val) + "\t" + String(tbtps2_val) + "\t" + String(tbAvg) + "\t" + String(throttlePos));
      //read and calculate current motor position
      long tbTemp1 = abs(((analogRead(TBTPS1_pin) - tbtps1_CLOSE) * 255) / tbtps1_range);
      long tbTemp2 = abs(((tbtps2_CLOSE - analogRead(TBTPS2_pin)) * 255) / tbtps2_range);
      //average between the two, use
      tbAvg = abs((tbTemp1 + tbTemp2) / 2);
      writeMot(0);
    }
  }

  //pedal tps sensor
  //read and calculate current pedal position
  long pTemp1 = abs(((analogRead(PTPS1_pin) - ptps1_CLOSE) * 255) / ptps1_range);
  long pTemp2 = abs(((analogRead(PTPS2_pin) - ptps2_CLOSE) * 255) / ptps2_range);
  //average between the two, use
  throttlePos = abs((pTemp1 + pTemp2) / 2);

  Serial.println(String(pTemp1) + "\t" + String(pTemp2) + "\t" + String(throttlePos));


}
/*
   writes to motor
   val: 9 bit, 0-127 is close, 129-255 is open
*/
void writeMot(long val)
{
  if (val > 255 || val < -1)
  {
    Serial.println("writeMot: ERROR VAL EXCEEDED " + String(val));
    analogWrite(EN_pin, 0);
    while (1);
  }
  else if (val == -1)
  {
    //hold close
    digitalWrite(IN2_pin, LOW);
    digitalWrite(IN1_pin, HIGH);
    analogWrite(EN_pin, CLOSE_VAL);
  }
  else
  {
    //open
    digitalWrite(IN2_pin, HIGH);
    digitalWrite(IN1_pin, LOW);
    analogWrite(EN_pin, val);
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
    mot_pwm = 100 + (2 * (throttlePos - tbAvg));
  }
  else if (tbAvg > throttlePos)
  {
    mot_pwm = 0;
  }
  mot_pwm = constrain(mot_pwm, 0, 255);
  //Serial.println(String(mot_pwm) + "\t" + String(tbAvg));
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
  writeMot(255);
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
  writeMot(0); //relax motor

  //////////////////EPEDAL SETUP
  Serial.println("Setting ePedal min. Release throttle, press s to continue");
  char a = 'a';
  while (a != 's')
  {
    a = Serial.read();
  }
  ptps1_CLOSE = analogRead(PTPS1_pin);
  ptps2_CLOSE = analogRead(PTPS2_pin);

  Serial.println("Setting ePedal min. Depress throttle all the way, press s to continue");
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
  // put your setup code  here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(20000000000);
  pinMode(IN1_pin, OUTPUT);
  pinMode(IN2_pin, OUTPUT);
  initSequence();

  //u_timer.every(INUM, setPos);
}
void loop() {
  setPos();
}
