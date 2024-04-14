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

double Kp = .1, Ki = 0.01, Kd = 1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&tbAvg, &mot_pwm, &throttlePos, Kp, Ki, Kd, DIRECT);



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
   abstracts writeMot
   sets position of throttle body based on global throttlePos
   interval controller so call on interval
*/
bool setPos(void *)
{
  readTPS();
  myPID.Compute();
  Serial.println(String(mot_pwm) + "\t" + String(tbAvg));
  writeMot(mot_pwm);
  return true;
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

  Serial.println("autotune running");
  PIDAutotuner tuner = PIDAutotuner();

  // Set the target value to tune to
  // This will depend on what you are tuning. This should be set to a value within
  // the usual range of the setpoint. For low-inertia systems, values at the lower
  // end of this range usually give better results. For anything else, start with a
  // value at the middle of the range.
  tuner.setTargetInputValue(throttlePos);

  // Set the loop interval in microseconds
  // This must be the same as the interval the PID control loop will run at
  tuner.setLoopInterval(1000);

  // Set the output range
  // These are the minimum and maximum possible output values of whatever you are
  // using to control the system (Arduino analogWrite, for example, is 0-255)
  tuner.setOutputRange(0, 255);

  // Set the Ziegler-Nichols tuning mode
  // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
  // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
  // safest option.
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

  // This must be called immediately before the tuning loop
  // Must be called with the current time in microseconds
  tuner.startTuningLoop(micros());

  // Run a loop until tuner.isFinished() returns true
  long microseconds;
  while (!tuner.isFinished()) {

    // This loop must run at the same speed as the PID control loop being tuned
    long prevMicroseconds = microseconds;
    microseconds = micros();

    // Get input value here (temperature, encoder position, velocity, etc)
    readTPS();
    //double input = doSomethingToGetInput();   tbAvg

    // Call tunePID() with the input value and current time in microseconds
    double mot_pwm = tuner.tunePID(tbAvg, microseconds);

    // Set the output - tunePid() will return values within the range configured
    // by setOutputRange(). Don't change the value or the tuning results will be
    // incorrect.
    //doSomethingToSetOutput(output);
    writeMot(mot_pwm);

    // This loop must run at the same speed as the PID control loop being tuned
    while (micros() - microseconds < 1000) delayMicroseconds(INUM);
  }

  // Turn the output off here.
  writeMot(0);

  // Get PID gains - set your PID controller's gains to these
  Kp = tuner.getKp();
  Ki = tuner.getKi();
  Kd = tuner.getKd();

  Serial.println("AUTOTUNE FINISHED");
  Serial.println("Kp: " + String(Kp) + "\t" + "Ki: " + String(Ki) + "\t" + "Kd: " + String(Kd));


  u_timer.every(INUM, setPos);
  myPID.SetSampleTime(1);
  myPID.SetMode(AUTOMATIC); //PID on
}
void loop() {
  u_timer.tick();
}
