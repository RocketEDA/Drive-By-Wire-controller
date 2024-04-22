#include <arduino-timer.h>
#include <PID_v1.h>
#include <pidautotuner.h>

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

#define INUM 10000  //PID loop interval in uS

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
  ptps1_val = analogRead(PTPS1_pin);
  ptps2_val = analogRead(PTPS2_pin);


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
   read max/min TB values
*/
void initSequence()
{
  Serial.println("initialization sequence");
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


}

void setup() {
  // put your setup code  here, to run once:
  Serial.begin(115200);

  pinMode(IN1_pin, OUTPUT);
  pinMode(IN2_pin, OUTPUT);
  initSequence();

  for (int i = 0; i < 255; i++)
  {
    writeMot(i);
    delay(500);
    readTPS();
    Serial.println(String(i) + "\t" + String(tbAvg));
  }
}
void loop() {
}
