/**********************************************************************
SINGLE-AXIS GIMBAL CONTROLLER
For Elegoo two-wheel balancing robot converted to gimbal

Core functionality only:
- Read IMU (body angle)
- Read encoders (wheel angle)
- PID control to keep wheel level
- Motor output

**********************************************************************/

#include <math.h>
#include <Wire.h>
#include "PicoEncoder-cga.h"
#include <LSM6DSOX.h>
#include <pico_encoder.pio.h>

/**********************************************************************
PIN DEFINITIONS (from original motor.h)
**********************************************************************/
#define AIN1 D7
#define PWMA_LEFT D5
#define BIN1 D12
#define PWMB_RIGHT D6
#define STBY_PIN D8
#define MAX_COMMAND 255
#define DEAD_ZONE 30
#define DEAD_ZONE_BOOST 10

// Encoder pins
const int left_a_pin = A0;
const int right_a_pin = D3;  // D3 maps to pin 3 on RP2040

/**********************************************************************
MOTOR CONSTANTS
**********************************************************************/
#define ENCODER_TO_RADIANS ((2*3.14159265)/((float) 11*4*30*64))
#define SERVO_INTERVAL 1000  // microseconds (1kHz control loop)

// IMU scaling
#define ASCALE (2*9.81/(2<<15))
#define ACC_TO_RAD (0.1176)    // Calibrate this!
#define GSCALE (3.14159*250/(180*32768))
#define HALF_TS_GSCALE (0.5*0.001*GSCALE)

// Safety
#define WHEEL_ANGLE_LIMIT 0.5  // radians

/**********************************************************************
GLOBAL VARIABLES
**********************************************************************/
// IMU offsets - TODO: CALIBRATE
int ay0 = -225;   // Accelerometer Y offset
int gx0 = -60;     // Gyroscope X offset

// Control gains - TUNE THESE
float Kp_wheel = 80.0;      // Wheel position gain
float Kd_wheel = 5.0;      // Wheel velocity gain (damping)
float Kf_body = 0.4;        // Kalman filter gain

// State variables
float body_angle = 0;
float last_body_angle = 0;
int last_gx = 0;
int ay_raw = 0;
int gx_raw = 0;

// Motor enable
bool go = false;

// Encoder objects
PicoEncoder left_encoder;
PicoEncoder right_encoder;

/**********************************************************************
MOTOR FUNCTIONS
**********************************************************************/
void motor_init() {
    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMA_LEFT, OUTPUT);
    pinMode(PWMB_RIGHT, OUTPUT);
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, LOW);
    analogWriteFreq(20000);
    analogWrite(PWMA_LEFT, 0);
    analogWrite(PWMB_RIGHT, 0);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
}

void motor_stop() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, LOW);
    analogWrite(PWMA_LEFT, 0);
    analogWrite(PWMB_RIGHT, 0);
}

int compensate_command(int command) {
    if (command == 0) return 0;
    if (abs(command) < DEAD_ZONE) return 0;
    
    // boost to overcome stiction
    int compensation = command + (command > 0 ? 
                       DEAD_ZONE_BOOST : -DEAD_ZONE_BOOST);
    // saturate
    if (compensation > MAX_COMMAND) compensation = MAX_COMMAND;
    if (compensation < -MAX_COMMAND) compensation = -MAX_COMMAND;

    return compensation;
}

void motor_command(int speed) {
    speed = compensate_command(speed);

    if (speed >= 0) {
        digitalWrite(AIN1, 0);
        digitalWrite(BIN1, 0);
        analogWrite(PWMA_LEFT, speed > 255 ? 255 : speed);
        analogWrite(PWMB_RIGHT, speed > 255 ? 255 : speed);
    } else {
        digitalWrite(AIN1, 1);
        digitalWrite(BIN1, 1);
        analogWrite(PWMA_LEFT, speed < -255 ? 255 : -speed);
        analogWrite(PWMB_RIGHT, speed < -255 ? 255 : -speed);
    }
}

/**********************************************************************
IMU FUNCTIONS
**********************************************************************/
void imu_init() {
    Wire.begin();
    Wire.setClock(1000000UL);
    if (!IMU.begin()) {
        Serial.println("IMU failed!");
        while (1) delay(1000);
    }
}

void imu_read() {
    int ax_raw, az_raw, gy_raw, gz_raw;
    if (IMU.accelerationAvailable())
        IMU.readAcceleration(ax_raw, ay_raw, az_raw);
    if (IMU.gyroscopeAvailable())
        IMU.readGyroscope(gx_raw, gy_raw, gz_raw);
}

/**********************************************************************
ENCODER FUNCTIONS
**********************************************************************/
void encoder_init() {
    left_encoder.begin(left_a_pin);
    right_encoder.begin(right_a_pin);
    // Update these for YOUR robot
    left_encoder.setPhases(0x422670);
    right_encoder.setPhases(0x353D55);
}

void encoder_read(float &wheel_angle, float &wheel_velocity) {
    left_encoder.update();
    right_encoder.update();
    
    int left_angle = +left_encoder.position;
    int right_angle = +right_encoder.position;
    int left_speed = +left_encoder.speed;
    int right_speed = +right_encoder.speed;
    
    wheel_angle = 0.5 * ENCODER_TO_RADIANS * (left_angle + right_angle);
    wheel_velocity = 0.5 * ENCODER_TO_RADIANS * (left_speed + right_speed);
}

/**********************************************************************
BODY ANGLE ESTIMATION (Kalman Filter)
**********************************************************************/
float estimate_body_angle() {
    int ay = ay_raw - ay0;
    int gx = gx_raw - gx0;
    
    // Prediction step
    float predicted = last_body_angle + HALF_TS_GSCALE * (gx + last_gx);
    
    // Measurement update (from accelerometer)
    float accel_angle = ASCALE * ACC_TO_RAD * ay;
    float estimated = predicted - Kf_body * (predicted - accel_angle);
    
    last_body_angle = estimated;
    last_gx = gx;
    
    return estimated;
}

/**********************************************************************
USER INPUT
**********************************************************************/
void process_user_input() {
    if (Serial.available() <= 0) return;
    int c = Serial.read();
    switch (c) {
        case 'G': case 'g':
            Serial.println("Go!");
            go = true;
            break;
        case 'S': case 's':
            Serial.println("Stop!");
            go = false;
            break;
    }
}

/**********************************************************************
SETUP
**********************************************************************/
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("=== SINGLE AXIS GIMBAL TEST ===");
    delay(1000);
    
    motor_init();
    Serial.println("Motors ready");
    
    imu_init();
    Serial.println("IMU ready");
    
    encoder_init();
    Serial.println("Encoders ready");
    
    // Wait for user
    Serial.println("Type 'g' to start, 's' to stop");
}

/**********************************************************************
MAIN CONTROL LOOP
**********************************************************************/
// void loop() {
//     for (int i = 0; i < 256; i+= 5) {
//         Serial.printf("commanding %d\n", i);
//         motor_command(i);
//         delay(3000);
//     }
//     motor_command(0);
//     exit(0);
// }

void loop() {
    // Wait for go command
    if (!go) {
        motor_stop();
        process_user_input();
        delay(50);
        return;
    }
    
    static unsigned long last_time = 0;
    unsigned long now = micros();
    
    // Run at 1kHz (SERVO_INTERVAL = 1000 microseconds)
    if (now - last_time < SERVO_INTERVAL) {
        process_user_input();  // Still check for stop command
        return;
    }
    last_time = now;
    
    // 1. Read sensors
    imu_read();
    float wheel_angle, wheel_velocity;
    encoder_read(wheel_angle, wheel_velocity);
    
    // 2. Estimate body angle (disturbance)
    body_angle = estimate_body_angle();
    
    // 3. THE KEY INSIGHT: 
    //    To keep wheel level relative to ground,
    //    wheel angle should EQUAL -body_angle
    float wheel_desired = -body_angle;
    
    // 4. Control law: follow the desired wheel angle
    float error = wheel_desired - wheel_angle;
    float error_velocity = -wheel_velocity;  // Derivative of error
    
    float command = 
        Kp_wheel * error           // Position error: wheel not where it should be
        + Kd_wheel * error_velocity; // Velocity error: wheel moving wrong way
    
    // 5. Apply command to motors
    motor_command((int)command);
    
    // 6. Debug output (every ~200ms)
    static unsigned long debug_timer = 0;
    if (micros() - debug_timer > 200000) {
        debug_timer = micros();
        Serial.print("Body: ");
        Serial.print(body_angle * 180/3.14159, 1);
        Serial.print("°  Wheel: ");
        Serial.print(wheel_angle * 180/3.14159, 1);
        Serial.print("°  Desired: ");
        Serial.print(wheel_desired * 180/3.14159, 1);
        Serial.print("°  Error: ");
        Serial.print(error * 180/3.14159, 1);
        Serial.print("°  Cmd: ");
        Serial.println(command);
    }
    
    // Check for stop command
    process_user_input();
}