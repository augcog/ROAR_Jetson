#include <EnableInterrupt.h>
#include <Servo.h>

const float pi = 3.14159265359;
const float d = 0.0666;
float velocity = 0;
float encoder_count = 0;
int received_pwm_signal = 0;
int pwm_esc = 1499;
int pwm_steer = 1499;
int first_signal = 1;
int loopcount = 1;
volatile unsigned long dt = 0;
volatile unsigned long lastflag = 0;
volatile unsigned long current_time = 0;
float throttleIn = 1500; //global variable for throttle
float steeringIn = 1500; //global variable for steering

class Car {
  public:

    void initArduino();
    void readReceiver();
    void writeToActuators();
    void encoder_setup();
    void check_flags();

    void Car::interfunc() {
      encoder_time_old = encoder_time;
      encoder_time = micros();
      dt = encoder_time - encoder_time_old;
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += 1;
    }

    void calc_velocity() {
      velocity = 0.125 * pi * d * 1000000 / dt;
      current_time = millis();
      if (current_time - lastflag > 50) {
        velocity = 0;
      }
    }

  private:
    const int SERVO_PIN = 10;
    const int MOTOR_PIN = 11;

    const int ch1 = 8;
    const int ch2 = 9;

    Servo throttle;
    Servo steering;

    const int MOTOR_MAX = 1750;
    const int MOTOR_MIN = 800;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 3000;
    const int THETA_MIN = 0;

    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);

    int encoder_count_old = 0;

    int phasestate1 = 0;


    const int ENCODER_FLAG = 1;

    //    unsigned long encoder_dt = 0;
    volatile unsigned long encoder_time = 0;
    volatile unsigned long encoder_time_old = 0;

    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
};

Car car;

void interfuncCB() {
  car.interfunc();
}

void setup() {

  car.initArduino();
  car.encoder_setup();
  delay(1500);
  Serial.begin(9600);

}

void loop() {

  car.check_flags();
  car.calc_velocity();

  car.readReceiver();
  if (steeringIn < 500) {
    // recieve throttle information from Jetson Nano
  } else {
    car.writeToActuators();
  }

  Serial.print("Throttle PWM: ");
  Serial.print(throttleIn, 4);
  Serial.print(", Throttle PWM: ");
  Serial.print(steeringIn, 4);
  Serial.print(", vel: ");
  Serial.print(velocity, 4);
  Serial.print(", enc_cnt: ");
  Serial.print(encoder_count);
  Serial.println();

}

void Car::encoder_setup() {
  pinMode(4, INPUT_PULLUP); // attach pin for phase 3 to pin 4
  enableInterrupt(4, interfuncCB, CHANGE);
}

void Car::check_flags() {
  if (updateFlagsShared) {
    lastflag = millis();
    noInterrupts();

    updateFlags = updateFlagsShared;

    updateFlagsShared = 0;
    interrupts();
  }
}

void Car::initArduino() {
  throttle.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
  pinMode(ch2, INPUT);
  pinMode(ch1, INPUT);
}

void Car::readReceiver() {
  throttleIn = pulseIn (ch2, HIGH);
  steeringIn = pulseIn (ch1, HIGH);
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

void Car::writeToActuators() {
  throttle.writeMicroseconds( (uint16_t) saturateMotor( throttleIn ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( steeringIn ) );
}
