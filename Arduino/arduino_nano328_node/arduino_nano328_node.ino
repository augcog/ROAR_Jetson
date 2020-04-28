#include <ros.h>
#include <Servo.h>
#include <std_msgs/Float32.h>
#include <EnableInterrupt.h>
#include <barc/ECU.h>
#include <barc/Encoder.h>

// GLOBAL VARIABLES //
const float pi = 3.14159265359;
const float R = 0.0325;
const float C = 0.33;

volatile unsigned long dt;
volatile unsigned long t0;
volatile unsigned long ecu_t0;

int received_ecu_signal = 0;

class Car {
  public:
    // Brushless encoder functions
    void initEncoders();
    void readEncoderPins();
    int getEncoderCounts();

    void phase1interfunc();
    void phase2interfunc();
    void phase3interfunc();

    // Used for copying variables shared with interrupts to avoid read/write
    // conflicts later
    void readAndCopyInputs();

    // Velocity estimation
    void calcVelocity();
    float getVelocity();

    // Actuator functions
    void initActuators();
    void armActuators();
    void writeToActuators(const barc::ECU& ecu);
    void killMotor();

  private:
    const int ENCODER_FLAG = 1;

    float velocity = 0;
    int vel_dir;
    int encoder_count = 0;
    int encoder_count_old = 0;
    int encoder_delta_count = 0;
    unsigned long encoder_time = 0;
    unsigned long encoder_time_old = 0;
    unsigned long encoder_delta_time = 0;
    uint8_t updateFlags;

    // Variables used in interrupt callback functions
    volatile int dir;
    volatile int phase_state_1 = 0;
    volatile int phase_state_2 = 0;
    volatile int phase_state_3 = 0;
    volatile int encoder_count_shared = 0;
    volatile unsigned long encoder_time_shared = 0;
//    volatile unsigned long encoder_time_old = 0;
    volatile uint8_t updateFlagsShared;

    // Pin assignments
    const int ENC_P1_PIN = 2;
    const int ENC_P2_PIN = 3;
    const int ENC_P3_PIN = 4;
    const int MOTOR_PIN = 10;
    const int SERVO_PIN = 11;

    // Actuator limits
    const int MOTOR_MAX = 1900;
    const int MOTOR_MIN = 1100;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 1900;
    const int THETA_MIN = 1100;

    // Interfaces to motor and steering actuators
    Servo motor;
    Servo steering;

    // Utility functions
    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

Car car;

void phase1interfuncCB() {
  car.phase1interfunc();
}

void phase2interfuncCB() {
  car.phase2interfunc();
}

void phase3interfuncCB() {
  car.phase3interfunc();
}

void ecuCallback(const barc::ECU& ecu) {
  car.writeToActuators(ecu);
  received_ecu_signal = 1;
}

// Setup ROS publishers and subscribers
ros::NodeHandle nh;

// Global message variables
// Encoder, RC Inputs, Electronic Control Unit, Ultrasound
barc::ECU ecu;
barc::ECU rc_inputs;
barc::Encoder encoder;
barc::Encoder vel_est;

ros::Publisher pub_encoder("encoder", &encoder);
ros::Publisher pub_vel_est("vel_est", &vel_est);
ros::Subscriber<barc::ECU> sub_ecu("ecu_pwm", ecuCallback);

void setup() {
  // Set up encoders, and actuators
  car.initEncoders();
  car.initActuators();

  nh.getHardware()->setBaud(9600);

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  nh.advertise(pub_encoder);
  nh.advertise(pub_vel_est);
  nh.subscribe(sub_ecu);

  // Arming ESC, 1 sec delay for arming and ROS
  car.armActuators();
  t0 = millis();
  ecu_t0 = millis();
}

void loop() {
  dt = millis() - t0;

//  nh.spinOnce();

  if ( (millis() - ecu_t0) >= 1000) {
    if (!received_ecu_signal) {
      car.killMotor();
    } else {
      received_ecu_signal = 0;
    }
    ecu_t0 = millis();
  }
//  nh.spinOnce();

  if (dt > 50) {
    car.readAndCopyInputs();

    // publish velocity estimate
    car.calcVelocity();
    vel_est.FL  = 0.0;
    vel_est.FR  = 0.0;
    vel_est.BL  = 0.0;
    vel_est.BR  = 0.0;
    vel_est.DS  = car.getVelocity();
    pub_vel_est.publish(&vel_est);

    // publish encoder ticks
    encoder.FL = 0;
    encoder.FR = 0;
    encoder.BL = 0;
    encoder.BR = 0;
    encoder.DS = car.getEncoderCounts();
    pub_encoder.publish(&encoder);

    t0 = millis();
  }

  nh.spinOnce();
}

/**************************************************************************
CAR CLASS IMPLEMENTATION
**************************************************************************/
void Car::initEncoders() {
  pinMode(ENC_P1_PIN, INPUT_PULLUP); // attach pin for phase 1 to pin 2
  pinMode(ENC_P2_PIN, INPUT_PULLUP); // attach pin for phase 2 to pin 3
  pinMode(ENC_P3_PIN, INPUT_PULLUP); // attach pin for phase 3 to pin 4
  enableInterrupt(ENC_P1_PIN, phase1interfuncCB, CHANGE);
  enableInterrupt(ENC_P2_PIN, phase2interfuncCB, CHANGE);
  enableInterrupt(ENC_P3_PIN, phase3interfuncCB, CHANGE);
}

void Car::readAndCopyInputs() {
  // check shared update flags to see if any channels have a new signal
  if (updateFlagsShared) {
    // Turn off interrupts, make local copies of variables set by interrupts,
    // then turn interrupts back on. Without doing this, an interrupt could
    // update a shared multibyte variable while the loop is in the middle of
    // reading it
    noInterrupts();

    updateFlags = updateFlagsShared;
    if (updateFlags & ENCODER_FLAG) {
      encoder_count = encoder_count_shared;
      encoder_time = encoder_time_shared;
      //encoder_delta_time = encoder_time - encoder_time_old;
      vel_dir = dir;
    }

    updateFlagsShared = 0;
    interrupts();
  }
}

void Car::readEncoderPins() {
  phase_state_1 = digitalRead(ENC_P1_PIN);
  phase_state_2 = digitalRead(ENC_P2_PIN);
  phase_state_3 = digitalRead(ENC_P3_PIN);
}

void Car::phase1interfunc() {
  readEncoderPins();
  if (phase_state_1 + phase_state_2 + phase_state_3 == 2) {
    if (phase_state_2 == HIGH) {
      dir = -1;
    } else {
      dir = 1;
    }
  } else {
    if (phase_state_2 == HIGH) {
      dir = 1;
    } else {
      dir = -1;
    }
  }
  //encoder_time_old = encoder_time;
  encoder_time_shared = micros();
  updateFlagsShared |= ENCODER_FLAG;
  encoder_count_shared -= dir;
}

void Car::phase2interfunc() {
  readEncoderPins();
  if (phase_state_1 + phase_state_2 + phase_state_3 == 2) {
    if (phase_state_1 == HIGH) {
      dir = 1;
    } else {
      dir = -1;
    }
  } else {
    if (phase_state_1 == HIGH) {
      dir = -1;
    } else {
      dir = 1;
    }
  }
  //encoder_time_old = encoder_time;
  encoder_time_shared = micros();
  updateFlagsShared |= ENCODER_FLAG;
  encoder_count_shared -= dir;
}

void Car::phase3interfunc() {
  readEncoderPins();
  if (phase_state_1 + phase_state_2 + phase_state_3 == 2) {
    if ( phase_state_1 == HIGH) {
      dir = -1;
    } else {
      dir = 1;
    }
  } else {
    if ( phase_state_1 == HIGH) {
      dir = 1;
    } else {
      dir = -1;
    }
  }
  //encoder_time_old = encoder_time;
  encoder_time_shared = micros();
  updateFlagsShared |= ENCODER_FLAG;
  encoder_count_shared -= dir;
}

int Car::getEncoderCounts() {
  return encoder_count;
}

void Car::calcVelocity() {
  if (encoder_count_old != encoder_count) {
    encoder_delta_count = encoder_count - encoder_count_old;
    encoder_delta_time = encoder_time - encoder_time_old;
    //velocity = -vel_dir*0.25*pi*R/(encoder_delta_time/1000000.0);
    // Hack-ish estimate of 112 encoder counts per revolution of the wheels assuming
    // locked diff. Need to get number of encoder counts per revolution of motor shaft
    // and gear ratio between motor output and drive shaft
    // velocity = -vel_dir*(2*pi*R*encoder_delta_count/112.0)/(encoder_delta_time/1000000.0);
    velocity = -vel_dir*(C*encoder_delta_count/113.0)/(encoder_delta_time/1000000.0);
  }
  else {
    velocity = 0.0;
  }
  encoder_count_old = encoder_count;
  encoder_time_old = encoder_time;
}

float Car::getVelocity() {
  return velocity;
}

float Car::saturateMotor(float x) {
  if (x > MOTOR_MAX) { x = MOTOR_MAX; }
  if (x < MOTOR_MIN) { x = MOTOR_MIN; }
  return x;
}

float Car::saturateServo(float x) {
  if (x > THETA_MAX) { x = THETA_MAX; }
  if (x < THETA_MIN) { x = THETA_MIN; }
  return x;
}

void Car::initActuators() {
  motor.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
}

void Car::armActuators() {
  motor.writeMicroseconds(2000);
  delayMicroseconds(50);
  motor.writeMicroseconds( (uint16_t) MOTOR_NEUTRAL );
  steering.writeMicroseconds( (uint16_t) THETA_CENTER );
  delay(1000);
}

void Car::writeToActuators(const barc::ECU& ecu) {
  motor.writeMicroseconds( (uint16_t) saturateMotor(ecu.motor) );
  steering.writeMicroseconds( (uint16_t) saturateServo(ecu.servo) );
}

uint16_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  // Mapping from microseconds to pwm angle
  // 0 deg -> 1000 us , 90 deg -> 1500 us , 180 deg -> 2000 us
  // ref: camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino

  // saturate signal
  if(microseconds > 2000 ){ microseconds = 2000; }
  if(microseconds < 1000 ){ microseconds = 1000; }

  // map signal from microseconds to pwm angle
  uint16_t pwm = (microseconds - 1000.0)/1000.0*180;
  return static_cast<uint8_t>(pwm);
}

void Car::killMotor(){
  motor.writeMicroseconds( (uint16_t) MOTOR_NEUTRAL );
  steering.writeMicroseconds( (uint16_t) THETA_CENTER );
}