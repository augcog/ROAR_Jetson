#include <Servo.h>
#include <EnableInterrupt.h>

Servo throttle;
Servo steer;

const int throttle_pin = 11;  // identify the pins that each servo signal wire is connected to
const int steer_pin = 10;

unsigned long now;
unsigned long rc_update;

class Car {
  public:
    void initRCInput();

    uint16_t getRCThrottle();
    uint16_t getRCSteering();


    void readAndCopyInputs();
    // Actuator functions
    void initActuators();
    void armActuators();
    void writeToActuators();
    void killMotor();

  private:
    const int THROTTLE_PIN = 7;
    const int STEERING_PIN = 8;

    const int MOTOR_MAX = 2000;
    const int MOTOR_MIN = 800;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 1900;
    const int THETA_MIN = 1100;

    Servo motor;
    Servo steering;

    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
    const int THROTTLE_FLAG = 1;
    const int STEERING_FLAG = 2;

    uint32_t throttleStart;
    uint32_t steeringStart;
    volatile uint16_t throttleInShared;
    volatile uint16_t steeringInShared;
    uint16_t throttleIn = 1500;
    uint16_t steeringIn = 1500;


    float throttle_neutral_ms = 1500.0;
    float servo_neutral_ms = 1500.0;

    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

float pi                = 3.141593;

Car car;

volatile unsigned long dt;
volatile unsigned long t0;
volatile unsigned long ecu_t0;

void setup() {

  throttle.attach(throttle_pin, 500, 2500); // attach the servo library to each servo pin, and define min and max uS values
  steer.attach(steer_pin, 500, 2500);

  car.initRCInput();

  car.armActuators();
  t0 = millis();

}

void loop() {

  // compute time elapsed (in ms)
  dt = millis() - t0;

  car.readAndCopyInputs();

  rc_inputs.motor = Car.getRCThrottle();
  rc_inputs.servo = Car.getRCSteering();

  t0 = millis();
}

float Car::saturateMotor(float x) {
  if (x > MOTOR_MAX) {
    x = MOTOR_MAX;
  }
  if (x < MOTOR_MIN) {
    x = MOTOR_MIN;
  }
  return x;
}

float Car::saturateServo(float x) {
  if (x > THETA_MAX) {
    x = THETA_MAX;
  }
  if (x < THETA_MIN) {
    x = THETA_MIN;
  }
  return x;
}

void Car::initRCInput() {
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);
  enableInterrupt(THROTTLE_PIN, calcThrottleCallback, CHANGE);
  enableInterrupt(STEERING_PIN, calcSteeringCallback, CHANGE);
}

void Car::initActuators() {
  motor.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
}

void Car::armActuators() {
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
  delay(1000);
}

void Car::writeToActuators() {
  motor.writeMicroseconds( (uint16_t) saturateMotor( ecu.motor ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( ecu.servo ) );
}

uint16_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  // Mapping from microseconds to pwm angle
  // 0 deg -> 1000 us , 90 deg -> 1500 us , 180 deg -> 2000 us
  // ref: camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino

  // saturate signal
  if (microseconds > 2000 ) {
    microseconds = 2000;
  }
  if (microseconds < 1000 ) {
    microseconds = 1000;
  }

  // map signal from microseconds to pwm angle
  uint16_t pwm = (microseconds - 1000.0) / 1000.0 * 180;
  return static_cast<uint8_t>(pwm);
}

void Car::calcThrottle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    // rising edge of the signal pulse, start timing
    throttleStart = micros();
  } else {
    // falling edge, calculate duration of throttle pulse
    throttleInShared = (uint16_t)(micros() - throttleStart);
    // set the throttle flag to indicate that a new signal has been received
    updateFlagsShared |= THROTTLE_FLAG;
  }
}

void Car::calcSteering() {
  if (digitalRead(STEERING_PIN) == HIGH) {
    steeringStart = micros();
  } else {
    steeringInShared = (uint16_t)(micros() - steeringStart);
    updateFlagsShared |= STEERING_FLAG;
  }
}

void Car::killMotor() {
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
}

void Car::readAndCopyInputs() {
  // check shared update flags to see if any channels have a new signal
  if (updateFlagsShared) {
    // Turn off interrupts, make local copies of variables set by interrupts,
    // then turn interrupts back on. Without doing this, an interrupt could
    // update a shared multibyte variable while the loop is in the middle of
    // reading it
    noInterrupts();
    // make local copies
    updateFlags = updateFlagsShared;
    if (updateFlags & THROTTLE_FLAG) {
      throttleIn = throttleInShared;
    }
    if (updateFlags & STEERING_FLAG) {
      steeringIn = steeringInShared;
    }
    // clear shared update flags and turn interrupts back on
    updateFlagsShared = 0;
    interrupts();
  }
}

uint16_t Car::getRCThrottle() {
  return throttleIn;
}
uint16_t Car::getRCSteering() {
  return steeringIn;
}
