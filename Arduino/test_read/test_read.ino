#include <Servo.h>

float throttleIn = 1500; //global variable for throttle
float steeringIn = 1500; //global variable for steering

class Car {
  public:
    void initArduino();
    void readReceiver();
    void writeToActuators();

  private:

    const int MOTOR_PIN = 9;
    const int SERVO_PIN = 10;

    const int ch1 = 4;
    const int ch2 = 5;

    Servo throttle;
    Servo steering;

    const int MOTOR_MAX = 2000;
    const int MOTOR_MIN = 800;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 3000;
    const int THETA_MIN = 0;

    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

Car car;

void setup() {
  car.initArduino();
}

//Main Program
void loop() {
  car.readReceiver();
  if (steeringIn < 500) {
    // recieve throttle information from Jetson Nano
  } else {
    car.writeToActuators();
  }
}

void Car::initArduino() {
  throttle.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
}

void Car::readReceiver() {
  throttleIn = pulseIn (ch1, HIGH);
  steeringIn = pulseIn (ch2, HIGH);
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

void Car::writeToActuators() {
  throttle.writeMicroseconds( (uint16_t) saturateMotor( throttleIn ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( steeringIn ) );
}
