bool TESTING_STATE = false; //test between receiving Nvidia Jetson Signal and Writing Constant PWM
const int BUFFER_SIZE = 100;
const char HANDSHAKE_START = '(';
const char HANDSHAKE_END = ')';
char DELIMETER = ',';
char receivedData[BUFFER_SIZE]; //creates variable to store data from jetson (100 is byte size)

float throttle_read = 1500;
float steering_read = 1500;
float throttle_write = 1500;
float steering_write = 1500;
float receiverStateVar = LOW;
float str_prev_2 = 1500;
float str_prev_1 = 1500;
float thr_prev_2 = 1500;
float thr_prev_1 = 1500;

#define BAUD_RATE 9600



#include "Servo.h"

// Assign your channel in pins
#define CHANNEL1_IN_PIN 2
#define CHANNEL2_IN_PIN 3
#define RECEIVER_STATE_PIN 30

// Assign your channel out pins
#define CHANNEL1_OUT_PIN 4
#define CHANNEL2_OUT_PIN 5

Servo servoChannel1;
Servo servoChannel2;

#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2

volatile uint32_t bUpdateFlagsShared;

volatile uint32_t unChannel1InShared;
volatile uint32_t unChannel2InShared;

void setup()
{
  Serial.begin(BAUD_RATE);  // Jetson
  Serial1.begin(BAUD_RATE); // bluetooth on PIN 17, 18

  servoChannel1.attach(CHANNEL1_OUT_PIN);
  servoChannel2.attach(CHANNEL2_OUT_PIN);

  pinMode(RECEIVER_STATE_PIN, INPUT);

  attachInterrupt(CHANNEL1_IN_PIN, calcChannel1, CHANGE);
  attachInterrupt(CHANNEL2_IN_PIN, calcChannel2, CHANGE);

}

void loop()
{
  receiverStateVar = digitalRead(RECEIVER_STATE_PIN);

  static uint32_t unChannel1In;
  static uint32_t unChannel2In;

  static uint32_t bUpdateFlags;

  if (bUpdateFlagsShared)
  {
    //noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;

    if (bUpdateFlags & CHANNEL1_FLAG)
    {
      if (unChannel1In > 2500) {
        unChannel1In = str_prev_1 - str_prev_2 + str_prev_1;
      }
      str_prev_2 = str_prev_1;
      str_prev_1 = unChannel1In;
      unChannel1In = unChannel1InShared;
    }

    if (bUpdateFlags & CHANNEL2_FLAG)
    {
      if (unChannel2In > 2500) {
        unChannel2In = thr_prev_1 - thr_prev_2 + thr_prev_1;
      }
      str_prev_2 = str_prev_1;
      str_prev_1 = unChannel1In;
      unChannel2In = unChannel2InShared;
    }
    bUpdateFlagsShared = 0;

    //interrupts();
  }

//    Serial.println();
//    Serial.print(unChannel1In);
//    Serial.print(",");
//    Serial.print(unChannel2In);


  throttle_read = unChannel2In;
  steering_read = unChannel1In;


  if (receiverStateVar == LOW) {
    throttle_write = throttle_read;
    steering_write = steering_read;
    writeToServo();
  } else if (receiverStateVar == HIGH) { // receiver is not connected
    if (TESTING_STATE == true) {
      throttle_write = 1500;
      steering_write = 1900;
    } else {
      // clear receivedData array first
      memset(receivedData, 0, BUFFER_SIZE);
      // Prioritize for Serial(Jetson) connection
      // Then is Serial1 (Bluetooth)
      if (Serial.available() > 0) {
          parseData("serial");
          writeToSerial();
        }
      else if(Serial1.available() > 0) {
        parseData("serial1");
        writeToSerial1(); 
      }

      // we need to write servo regardless of the mode
      writeToServo();
    }
      
      
  }

  bUpdateFlags = 0;
}

void parseData(String method){
    /* 
     *  method to parse received data given input method. 
     *  Currently supported channel are:
     *    Serial
     *    Serial1
     *  Messages are required to be in the format of HANDSHAKE_START PWMTHROTTLE STEERING HANDSHAKE_END
     *  For example, by default, HANDSHAKE_START= "(", HANDSHAKE_END=")"
     *  Then a sample receivedData could be (1500,1500)
    */
    if (method.compareTo("serial") == 0){
      size_t bytes_read = Serial.readBytesUntil(HANDSHAKE_END, receivedData, 100); //reads serial data into buffer and times out after 100ms
    } else if (method.compareTo("serial1") == 0){
      size_t bytes_read = Serial1.readBytesUntil(HANDSHAKE_END, receivedData, 100); //reads serial data into buffer and times out after 100ms
    }
    
    char *token = strtok(receivedData, ",");
    if (token[0] == HANDSHAKE_START) {
      if (token != NULL){
        throttle_read = atoi(token+1); // + 1 to skip the (
        throttle_write = throttle_read;
      }
      
      token = strtok(NULL, ",");
      if (token != NULL){
        steering_read = atoi(token);
        steering_write = steering_read;
      }
    }
}

void writeToSerial(){
  /*
   * Write to Serial
   */
    Serial.print(HANDSHAKE_START);
    Serial.print(throttle_write);
    Serial.print(",");
    Serial.print(steering_write);
    Serial.println(HANDSHAKE_END);  
}

void writeToSerial1() {
  /*
   * Write to Serial 1
   */
    Serial1.print(HANDSHAKE_START);
    Serial1.print(throttle_write);
    Serial1.print(",");
    Serial1.print(steering_write);
    Serial1.println(HANDSHAKE_END);  
}

void writeToServo(){
    servoChannel2.writeMicroseconds(throttle_write);
    servoChannel1.writeMicroseconds(steering_write);
}


void calcChannel1()
{
  static uint32_t ulStart;

  if (digitalRead(CHANNEL1_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel1InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL1_FLAG;
  }
}

void calcChannel2()
{
  static uint32_t ulStart;

  if (digitalRead(CHANNEL2_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel2InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL2_FLAG;
  }
}
