#include <Arduino.h>
#include <stdio.h>
#include <math.h>
static enum {topLeft_lim_switch, topRight_lim_switch, bottomLeft_lim_switch, bottomRight_lim_switch, top_lim_switch, right_lim_switch, left_lim_switch, bottom_lim_switch, null_switch} state = null_switch;


// Arduino PWM Speed Control：
const int pinINT0 = 2;  // Y CORD // left motat
const int pinINT1 = 3;  // X CORD // right motar
volatile int count_y = 0;
volatile int count_X = 0;
int E_X = 5;
int M_X = 4;
int E_Y = 6;
int M_Y = 7;
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

  pinMode(M_X, OUTPUT);
  pinMode(M_Y, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pinINT0), ISR_INT0, RISING);
  attachInterrupt(digitalPinToInterrupt(pinINT1), ISR_INT1, RISING);
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
int p_I_D_distance_TO_ENCODER_left (int target,volatile int &current){
  int error = target- current;
  int Kp = 50;
  int Ki=0;
  int Kd=0;
  int gain = error*Kp+Ki*1/error+Kd*error;
  return gain;
}
int p_I_D_distance_TO_ENCODER_right (int target,volatile int &current){
  int error = target- current;
  int Kp = 50;
  int Ki=0;
  int Kd=0;
  int gain = error*Kp+Ki*1/error+Kd*error;
  return gain;
}

void die_badday(){
    analogWrite(E_X, 0);
    analogWrite(E_Y, 0);
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
void line_basic(int encoder_count, volatile int &current_A, int M_A, int E_A,uint8_t state){
  while(abs(current_A)<abs(encoder_count)){
    Serial.print("THE A CORDINATE :");
    Serial.println(current_A);
    Serial.print(" THE CURRENT ENCODER VALUE : ");
    Serial.println(encoder_count);


    findState();
    process_state();
    digitalWrite(M_A, state);
    analogWrite(E_A, 255);
  }
    analogWrite(E_A, 0);
    exit(1);

}


void loop() {
    // digitalWrite(M_X, LOW);
    // analogWrite(E_X, 255);
    // digitalWrite(M_Y, LOW);
    // analogWrite(E_Y, 255); 
    // Serial.println(DistanceToMotorEncoder(50));
    line_basic(DistanceToMotorEncoder(50),count_X,M_X,E_X,LOW);
    while(1){
    findState();
    process_state();

    // Serial.print("The X cord: ");
    // Serial.println(coun_X);

    // Serial.print("The Y cord: ");
    // Serial.println(count_y);
    }

    // digitalWrite(M1, HIGH);
    // digitalWrite(M2, LOW);
    // analogWrite(E1, 200);
    // analogWrite(E2, 200); 

    // Serial.print("The Y cord: ");
    // Serial.println(countM2);
    // Serial.print("The X cord: ");
    // Serial.println(countM1);
    // // move_45(-1,1);
    // if (makePositive(float(countM1)) >500 || (makePositive(float(countM2)))  > 500){
    // while(1){
    // findState();
    // process_state();
    // analogWrite(E1, 0);
    // analogWrite(E2, 0); 
    //   }
    // }
}



void ISR_INT0() {
  if (digitalRead(M_Y) == HIGH) {
    count_y++;
  } else {
    count_y--;
  }
}

void ISR_INT1() {
  if (digitalRead(M_X) == HIGH) {
    count_X++;
  } else {
    count_X--;
  }
}

// Function to turn motor encoder into distance
int motorEncoderToMotorDistance(int encoderCounts) {
  int circum = 2 * PI * radius;
  int distance = encoderCounts * circum / encOneRev;
  return distance;
}
int DistanceToMotorEncoder(int distance){
  // int circum = 2 * PI * radius;
  // int encoderCounts =(distance * encOneRev)/ circum;
  return distance*50;
}
// Function to turn motor distance to actual distance
int* motorDistancetoActual() {
  static int results[2];  // Declare a static array to hold the results

  int distMot1 = motorEncoderToMotorDistance(count_y);
  int distMot2 = motorEncoderToMotorDistance(count_X);

  int x = (distMot1 + distMot2) / 2;
  int y = (distMot1 - distMot2) / 2;

  results[0] = x;
  results[1] = y;

  return results;  // Return a pointer to the array
}

// Functino that makes motor go up
void motorUp() {



    
  // if (digitalRead(topPin) ){
  //   analogWrite(E1, 0);
  //   analogWrite(E2, 0);
  //   exit(1);
  // } 
}