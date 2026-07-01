// motor params
#define POLE_PAIRS      11
// driver params
#define EN1             5
#define EN2             6
#define EN3             7
#define IN1             9
#define IN2             10
#define IN3             11
// encoder params
#define ENC_PWM         2
#define PWM_HZ          1000 // 1kHz
#define TOTAL_COUNTS    4119
#define MIN_COUNTS      16
#define MAX_COUNTS      4111
#define MAX_ENCODER_VALUE ((1 << 14) - 1) // 14-bit resolution
// other constants
#define BAUD_RATE   115200

#include <SimpleFOC.h>

MagneticSensorPWM sensor = MagneticSensorPWM(ENC_PWM, PWM_HZ, TOTAL_COUNTS, MIN_COUNTS, MAX_COUNTS);
void encoder_callback_fn() {sensor.handlePWM();}

BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, EN1, EN2, EN3);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  sensor.init();
  sensor.enableInterrupt(encoder_callback_fn);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 10; //set your power supply voltage;
  // limit the maximal dc voltage the driver can set
  driver.voltage_limit = 12; //set your voltage limit; Usually half of power supply voltage is a good 
  driver.enable_active_high = false;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
    driver.enable();

  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 6; //set your voltage limit in volts;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // set the target velocity [rad/s]
  motor.target = -6.28; // one rotation per second

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // loop FOC algorithm, should be called as 
  // frequently as possible for best 
  // performance (e.g. 1kHz+)
  motor.loopFOC();

  // open loop velocity movement
  motor.move();

  // user communication
  command.run();
}