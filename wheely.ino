#include "wheely.hpp"
#include "math.h"

#include <PID_v1.h>
#include <L298NX2.h>
#include <ServoInput.h>
#include <MPU6050_tockn.h>
#include <Wire.h>


int count = 0;
int loop_time = 0;
unsigned long last_update = 0;

double pid_in, pid_out, pid_target = 0;
double kp = 10, ki = 80, kd = 0.25; // good trimming for hi freq PWMs (490hz?)
// double kp = 10, ki = 200, kd = 0.25;

bool robot_down = false;
float in_rot=0, in_throttle=0, in_pot=0;

PID pid(&pid_in, &pid_out, &pid_target, kp, ki, kd, P_ON_E, REVERSE);

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

MPU6050 mpu6050(Wire);

TargetAccum accum(50, 0.50);


/* Signal pins for ServoInput MUST be interrupt-capable pins!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */
// Steering Setup
const int SteeringSignalPin = 1;  // MUST be interrupt-capable!
const int SteeringPulseMin = 948;  // microseconds (us)
const int SteeringPulseMax = 2008;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle Setup
const int ThrottleSignalPin = 0;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 960;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

// Pot Setup
const int Ch6SignalPin = 7;  // MUST be interrupt-capable!
const int Ch6PulseMin = 1040;  // microseconds (us)
const int Ch6PulseMax = 1932;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<Ch6SignalPin> pot(Ch6PulseMin, Ch6PulseMax);



void setup()
{
  pinMode(BUZZER, OUTPUT);

  // Used to display information
  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(PID_SAMPLE_TIME);
  pid.SetOutputLimits(-MAX_POWER, MAX_POWER);
  pid_target = 0;

  int wait_count=2;
	while ((wait_count--)>=0) {  // wait for all signals to be ready
		Serial.println("Waiting for servo signals...");
    if(ServoInput.available())
    {
      play_tone(link_up, LEN(link_up), 10);
      break;
    }
		play_tone(wait_link, LEN(wait_link));
    delay(300);
	}

  if(ServoInput.available() && steering.getPercent()>0.7 && throttle.getPercent()>0.7)
  { // TODO Save values to EEPROM. 
    play_tone(calibrating, LEN(calibrating));
    mpu6050.calcGyroOffsets(true);
    play_tone(calibrating, LEN(calibrating));
  }
  else
  {
    float offX = -2.17, offY = 2.44, offZ = 0.16;
    mpu6050.setGyroOffsets(offX, offY, offZ);
  }

  setPwmFrequency(EN_A, 256);
  setPwmFrequency(EN_B, 256);

  // tone(BUZZER, 300, 1000);
  // Wait for Serial Monitor to be opened
  // while (!Serial)
  // {
  //   //do nothing
  // }

  // delay(1000);
  play_tone(ready, LEN(ready));
}

void setMotors(int speedA, int speedB)
{
  speedA = constrain(speedA, -MAX_POWER, MAX_POWER);
  speedB = constrain(speedB, -MAX_POWER, MAX_POWER);
  if(abs(speedA)<MIN_POWER) speedA=0;
  if(abs(speedB)<MIN_POWER) speedB=0;


  motors.setSpeedA(abs(speedA));
  motors.setSpeedB(abs(speedB));

  speedA>=0 ? motors.forwardA(): motors.backwardA();
  speedB>=0 ? motors.forwardB(): motors.backwardB();
}

void loop()
{
  unsigned long st = millis();

  if(ServoInput.available())
  {
    in_rot = 2.0 * MAX_POWER * steering.getPercent() - MAX_POWER;
    in_throttle = 2.0 * PID_TARGET_RANGE * throttle.getPercent() - PID_TARGET_RANGE;
    in_pot = 2.0 * PID_TARGET_RANGE * pot.getPercent() - PID_TARGET_RANGE;

    pid_target = in_pot;

    if(abs(in_throttle)>1.0) // above throttle dead zone
    {
      pid_target += in_throttle;
    }
  }

  if (st - last_update >= PID_SAMPLE_TIME)
  {
    mpu6050.update();
    last_update = st;
    pid_in = mpu6050.getAngleX();
  }

  if (abs(pid_in) < MAX_ANGLE) // if not fallen
  {
    if(robot_down){
      robot_down=false;
      play_tone(ready, LEN(ready));
    }
    if (pid.Compute())
    {
      // set motor power after constraining it
      // motorPower = constrain(motorPower, -255, 255);
      int motorA = pid_out, motorB = pid_out;

      int rot = int(abs(in_rot));
      if(in_rot>=0)
      {
        motorA+=rot; motorB-=rot;
      }
      else
      {
        motorA-=rot; motorB+=rot;
      }

      setMotors(motorA, motorB);

      // if (accum.update((pid_out/MAX_POWER)*pid_in))
      // { // update target to account for balance shifts
      //   // only when no external input is applied
      //   // pid_target = -accum.getAccum();
      //   // accum.reset();
      // }
    }
  }
  else
  {
    accum.reset();
    pid_target = 0;
    setMotors(0, 0);
    if(!robot_down)
    {
      robot_down=true;
      play_tone(fallen, LEN(fallen));
    }
  }


  count += 1;
  loop_time += millis() - st;

  if (count >= 300)
  {
    
    float dt = loop_time / count;
    count = 0;
    loop_time = 0;
    Serial.print("Av time ");
    Serial.print(dt);
    Serial.print(" pid_out: ");
    Serial.print(pid_out);
    Serial.print(" pid_target: ");
    Serial.print(pid_target);
    Serial.print(" AngleX: ");
    Serial.print(pid_in);
    Serial.print(" Accum: ");
    Serial.println(accum.getAccum());

    Serial.print("IN R: "); Serial.print(in_rot);
    Serial.print(" T: "); Serial.print(in_throttle);
    Serial.print(" Pot: "); Serial.print(in_pot);
    Serial.println();
  }

}