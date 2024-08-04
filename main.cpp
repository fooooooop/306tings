#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
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


const int LED_pin = 13; //my funny ideas 

float radius = 14.78 / 2;  // mm

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



//timer setup for limit isr. 
cli();

TCCR1A = 0; 
TCCR1B = 0;
TCNT1 = 0; 
//testing here
unsigned int reload = 0xA3A; // 
TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);
OCR1A = reload;
TIMSK1 |= (1 << OCIE1A);
sei();
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
  while(1){
    analogWrite(E_X, 0);
    analogWrite(E_Y, 0);
    exit(1);
  }
      return;
}

int process_state() {
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
  return 1;
}

int p_I_D_distance_TO_ENCODER_left_y (int target,volatile int &current){
  int error = (target)*50- current; // 50 is an aprox conversion of encoders to distance 
  int Kp = 3;
  int Ki=0;
  int Kd=0;
  int gain = error*Kp+Ki*1/error+Kd*error;
  return gain;
}
int p_I_D_distance_TO_ENCODER_right_x (int target,volatile int &current){
  int error = (target*50)- current; // 50 is an aprox conversion of encoders to distance 
  int Kp = 3;
  int Ki=0;
  int Kd=0;
  int gain = error*Kp+Ki*1/error+Kd*error;
  return gain;
}
void move_right_x(int target, volatile int &count_A,  int accepted_range, int &M_A, int &E_A) {
    int gain_X = p_I_D_distance_TO_ENCODER_right_x(target, count_A);
    if (abs(gain_X) <= accepted_range) {
        die_badday();
        return;
    } else if (gain_X > 0) {
        digitalWrite(M_A, HIGH);
        analogWrite(E_A,gain_X);
    } else {
        digitalWrite(M_A, LOW);
        analogWrite(E_A, gain_X);
    }
    Serial.println(gain_X);
    return;
}


void pContorlRightMotor(float targetDistance, float Kp){

float targetCounts = distanceToMotorEncoder(targetDistance);

Serial.print("Target Counts: ");
Serial.println(targetCounts);


float motorSignal;
float error;

do{

  Serial.print("Target Counts: ");
  Serial.println(targetCounts);

  error = targetCounts - count_X;

  Serial.print("Current Count: ");
  Serial.println(count_X);


  Serial.print("Error: ");
   Serial.println(error);

  motorSignal = Kp*error;

  if (motorSignal >0){
    digitalWrite(M_X, HIGH);
    // Serial.print("Motor Signal: ");
    // Serial.println(motorSignal);
    analogWrite(E_X, constrain(motorSignal, 50,255));

  } else if (motorSignal < 0 ){
    digitalWrite(M_X, LOW);
    motorSignal = -motorSignal;

    // Serial.println("Motor Dir: LOW ");
    // Serial.print("Motor Signal: ");
    // Serial.println(motorSignal);
    analogWrite(E_X, constrain(motorSignal, 50,255));

  } else {
    motorSignal = 0;
    analogWrite(E_X, 0);
    die_badday();
    

  }




}while(abs(motorEncoderToMotorDistance(count_X) - targetDistance)>= 0.1*targetDistance);

analogWrite(E_X, 0);
die_badday();
Serial.println("Slay");


}


void loop() {
    // digitalWrite(M_X, LOW);
    // analogWrite(E_X, 0);
    // digitalWrite(M_Y, LOW);
    // analogWrite(E_Y, 0); 
    // Serial.println(DistanceToMotorEncoder(50));
    // p_I_D_distance_TO_ENCODER_right_x(50,count_x)
    // findState();
    // process_state();
    // digitalWrite(LED_pin, HIGH);
    // digitalWrite(LED_pin, LOW);

    pContorlRightMotor(50, 1.3);

    // Serial.print("Counts: ");
    // Serial.print(count_X);
    // Serial.print(" Distance: ");
    // Serial.println(motorEncoderToMotorDistance(count_X));

    // die_badday();


    // move_right_x(-50, count_X,10,M_X,E_X );
    // while(1){
    // findState();
    // process_state();

    // Serial.print("The X cord: ");
    // Serial.println(coun_X);

    // Serial.print("The Y cord: ");
    // Serial.println(count_y);
    // }

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
ISR(TIMER1_COMPA_vect) {
    findState();
    process_state();
    // Serial.println("why run this every time right? ");
}
// Function to turn motor encoder into distance
float motorEncoderToMotorDistance(float encoderCounts) {
  float circum = 2 * PI * radius;
  float distance = encoderCounts * circum / encOneRev;
  return distance;
}
float distanceToMotorEncoder(float distance){
  float circum = 2 * PI * radius;
  float encoderCounts =(distance * encOneRev)/ circum;
  return encoderCounts;
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
// void motorUp() {



    
//   // if (digitalRead(topPin) ){
//   //   analogWrite(E1, 0);
//   //   analogWrite(E2, 0);
//   //   exit(1);
//   // } 
// }