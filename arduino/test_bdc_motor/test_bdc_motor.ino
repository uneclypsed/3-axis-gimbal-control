#define BAUD_RATE 115200

#define STBY 2
#define AIN1 3
#define AIN2 4
#define PWMA 5
#define ENCA 18
#define ENCB 19

#define PPR_INTERNAL 11
#define GEAR_RATIO 30
#define PPR ((float)PPR_INTERNAL * (float)GEAR_RATIO)

#include <SimpleFOC.h>
#include <SimpleDCMotor.h>

// TODO: see about LinearHall in SimpleFOCDrivers repo
Encoder encoder = Encoder(ENCA, ENCB, PPR);

void encoder_handler_A() {encoder.handleA();}
void encoder_handler_B() {encoder.handleB();}

DCDriver1PWM2Dir driver = DCDriver1PWM2Dir(PWMA, AIN1, AIN2, STBY);

DCMotor motor = DCMotor();

void setup() {
  Serial.begin(BAUD_RATE);
  
  encoder.init();
  encoder.enableInterrupts(encoder_handler_A, encoder_handler_B);

  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit =  12.0f;
  // driver.pwm_frequency = 4096; // 4kHz 
  driver.init();
  
  motor.linkSensor(&encoder);
  motor.linkDriver(&driver);
  motor.voltage_limit = 12.0f;
  motor.controller = MotionControlType::angle;
  motor.init();
  motor.initFOC();
  motor.enable();
}

float target = 0;

void loop() {
  // motor.move(target);
  // target += PI / 4;
  // if (target >= 2 * PI) {
  //   target = 0;
  //   delay(2000);
  // }

  encoder.update();
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());


  // motor.move(0);
  // delay(5000);
  // motor.move(PI / 2);
  // Serial.print(encoder.getSensorAngle());
  // delay(5000);
  // motor.move(0);
  // motor.move(PI);
  // delay(5000);
  // motor.move(3 * PI / 2);
  // delay(5000);
  // motor.move(2 * PI);
  // delay(5000);
  // motor.disable();
  // exit(0);
}
