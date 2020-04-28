#include <Servo.h>
#include <EnableInterrupt.h>

const int chnl3_pin = 3;

class Car {
  public:

    void initActuators();
    void initRCInput();

    void no_movement();
    void writeToActuators();

    float steer_pwm = 1500.00;
    float throttle_pwm = 1500.00;

    uint16_t getRCThrottle();
    uint16_t getRCSteering();

    void calcThrottle();
    void calcSteering();

    void readAndCopyInputs();

  private:
    const int THROTTLE_FLAG = 1;
    const int STEERING_FLAG = 2;
    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
    uint32_t throttleStart;
    uint32_t steeringStart;
    volatile uint16_t throttleInShared;
    volatile uint16_t steeringInShared;
    uint16_t throttleIn = 1500;
    uint16_t steeringIn = 1500;

    const int MOTOR_PIN = 9;
    const int SERVO_PIN = 10;

    Servo throttle;
    Servo steering;

    const int THROTTLE_PIN = 4;
    const int STEERING_PIN  = 5;

    float neutral_pwm = 1500.00;

    const int MOTOR_MAX = 2000;
    const int MOTOR_MIN = 800;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 1900;
    const int THETA_MIN = 1100;

    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);

};

Car car;

void setup() {
  car.initActuators();
  car.initRCInput();
  Serial.begin(9600);
}

// introduces delay of 1 ms by measuring PWM rise and fall

void loop() {
  car.readAndCopyInputs();
  car.writeToActuators();
  Serial.print(car.getRCThrottle());
  Serial.print("  ");
  Serial.print(car.getRCSteering());
  Serial.println();
  if (digitalRead(chnl3_pin) == HIGH) {

    car.writeToActuators();

  } else {

    car.no_movement();

  }
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

void Car::initActuators() {
  throttle.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
}

void Car::initRCInput() {
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);
  enableInterrupt(THROTTLE_PIN, calcThrottleCallback, CHANGE);
  enableInterrupt(STEERING_PIN, calcSteeringCallback, CHANGE);
}

void Car::calcThrottle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    throttleStart = micros();
  } else {
    throttleInShared = (uint16_t)(micros() - throttleStart);
    updateFlagsShared |= THROTTLE_FLAG;
  }
  Serial.print("interrupt");
}

void Car::calcSteering() {
  if (digitalRead(STEERING_PIN) == HIGH) {
    steeringStart = micros();
  } else {
    steeringInShared = (uint16_t)(micros() - steeringStart);
    updateFlagsShared |= STEERING_FLAG;
  }
  Serial.print("interrupt");
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
    updateFlagsShared = 0;
    interrupts();
  }
}

void Car::no_movement() {
  throttle.writeMicroseconds( (uint16_t) neutral_pwm );
  steering.writeMicroseconds( (uint16_t) neutral_pwm );
}

void Car::writeToActuators() {
  throttle.writeMicroseconds( (uint16_t) saturateMotor( throttleIn ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( steeringIn ) );
}

void calcSteeringCallback() {
  car.calcSteering();
}
void calcThrottleCallback() {
  car.calcThrottle();
}

uint16_t Car::getRCThrottle() {
  return throttleIn;
}
uint16_t Car::getRCSteering() {
  return steeringIn;
}
