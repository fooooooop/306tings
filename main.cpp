#include <Arduino.h>
#include <stdio.h>
#include <math.h>
static enum {topLeft_lim_switch, topRight_lim_switch, bottomLeft_lim_switch, bottomRight_lim_switch, top_lim_switch, right_lim_switch, left_lim_switch, bottom_lim_switch, null_switch} state = null_switch;


// Arduino PWM Speed Control：
const int pinINT0 = 13;  // INT0 is on Pin 2 on Arduino Uno
volatile int countM1 = 0;
volatile int countM2 = 0;
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;
int encOneRev = 48 * 172 / 4;
int revRatio = 3 / 4;

int bottomPin = 9;
int topPin = 12;
int rightPin = 10;
int leftPin = 11;



int radius = 13.18 / 2;  // mm

void setup() {
  pinMode(pinINT0, INPUT_PULLUP);
  pinMode(bottomPin, OUTPUT);
  pinMode(topPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pinINT0), ISR_INT0, RISING);
  Serial.begin(115200);
}

void findState() {
    if (digitalRead(bottomPin) == HIGH && digitalRead(leftPin) == HIGH) {
        state = bottomLeft_lim_switch;
    } else if (digitalRead(bottomPin) == HIGH && digitalRead(rightPin) == HIGH) {
        state = bottomRight_lim_switch;
    } else if (digitalRead(topPin) == HIGH && digitalRead(leftPin) == HIGH) {
        state = topLeft_lim_switch;
    } else if (digitalRead(topPin) == HIGH && digitalRead(rightPin) == HIGH) {
        state = topRight_lim_switch;
    } else if (digitalRead(bottomPin) == HIGH) {
        state = bottom_lim_switch;
    } else if (digitalRead(topPin) == HIGH) {
        state = top_lim_switch;
    } else if (digitalRead(rightPin) == HIGH) {
        state = right_lim_switch;
    } else if (digitalRead(leftPin) == HIGH) {
        state = left_lim_switch;
    } else {
        state = null_switch;
    }
}

void die_badday(){
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    exit(1);
    return;
}

void process_state() {
    char input;
    switch (state) {
        case topLeft_lim_switch:
        Serial.println("topleft");
        die_badday();
            break;
        case topRight_lim_switch:
        Serial.println("topRight");
        die_badday();
            break;
        case bottomLeft_lim_switch:
        Serial.println("bottomLeft");
        die_badday();
            break;
        case bottomRight_lim_switch:
        Serial.println("bottomRight");
        die_badday();
            break;
        case top_lim_switch:
        Serial.println("top");
        die_badday();
            break;
        case right_lim_switch:
        Serial.println("right");
        die_badday();
            break;
        case left_lim_switch:
        Serial.println("left");
        die_badday();
            break;
        case bottom_lim_switch:
        Serial.println("bottom");
        die_badday();
            break;
        case null_switch:
        // Serial.println("null");
        // die_badday();
            break;
    }
}
float makePositive(float value) {
    if (value < 0) {
        return -value;
    } else {
        return value;
    }
}
void move_45(float cordX, float cordY){
  if(cordX>0){
    digitalWrite(M1, HIGH);
  }else if(cordX<0){
    digitalWrite(M1, LOW);
  } else{
    digitalWrite(M1, 0);
  }
  if(cordY>0){
     digitalWrite(M2, HIGH);
  }else if(cordY<0){
    digitalWrite(M2, LOW);
  } else{
    digitalWrite(M2, 0);
  }
  //motor power is applied at the end to not have uncorrect assiegnments
    analogWrite(E1, 255*makePositive(cordX));
    analogWrite(E2, 255*makePositive(cordY));
void rotate_angle(float* arr){
    //WARNING THIS WILL CONVER 0,1 => -0.707, 0.707
    float x1 = arr[0], y1 = arr[1];
    float x2 = 0, y2 = 0, angle = 0.785398; // 45 degrees in radians
    x2 = x1 * cos(angle) - y1 * sin(angle);
    y2 = x1 * sin(angle) + y1 * cos(angle);
    arr[0] = (round(x2*1000))/1000, arr[1] = (round(y2*1000))/1000;
}
void angle_to_coordinates(float angle, float& x, float& y) {
    float radians = angle * 22/7 / 180.0; //pie substatute 22/7
    x = cos(radians);
    y = sin(radians);
}
void move(float angle){
  float x,y;
  angle_to_coordinates(angle, x, y);
  float arr2[2] = {x, y};
  rotate_angle(arr2);
  x= arr2[0], y=arr2[1];
  move_45(x,y);
}

void loop() {
  // int* distances = motorDistancetoActual();
    // digitalWrite(M1, LOW);
    // digitalWrite(M2, LOW);
    // analogWrite(E1, 255);
    // analogWrite(E2, 255*0.95); 

  while(1){
    findState();
    process_state();
  }
}



void ISR_INT0() {
  if (digitalRead(M1) == HIGH) {
    countM1++;
  } else {
    countM1--;
  }
}

// Function to turn motor encoder into distance
int motorEncoderToMotorDistance(int encoderCounts) {
  int circum = 2 * PI * radius;
  int distance = encoderCounts * circum / encOneRev;
  return distance;
}

// Function to turn motor distance to actual distance
int* motorDistancetoActual() {
  static int results[2];  // Declare a static array to hold the results

  int distMot1 = motorEncoderToMotorDistance(countM1);
  int distMot2 = motorEncoderToMotorDistance(countM2);

  int x = (distMot1 + distMot2) / 2;
  int y = (distMot1 - distMot2) / 2;

  results[0] = x;
  results[1] = y;

  return results;  // Return a pointer to the array
}

// Functino that makes motor go up
void motorUp() {

    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 255);
    analogWrite(E2, 255);


    
  // if (digitalRead(topPin) ){
  //   analogWrite(E1, 0);
  //   analogWrite(E2, 0);
  //   exit(1);
  // } 
}
