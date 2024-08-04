#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
static enum {topLeft_lim_switch, topRight_lim_switch, bottomLeft_lim_switch, bottomRight_lim_switch, top_lim_switch, right_lim_switch, left_lim_switch, bottom_lim_switch, null_switch} state = null_switch;


// Arduino PWM Speed Control：
const int pinINT0 = 2;  // Y CORD // left motat
const int pinINT1 = 3;  // X CORD // right motar
volatile int count_Y = 0;
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
const float MOTOR_RIGHT_CONST_X =65.62;
const float MOTOR_LEFT_CONST_Y = 63.7;


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
        // die_badday();
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

void motor_power_ramp_down(float motorSignal_X,float motorSignal_Y,float error_X,float error_Y,int E_X, int E_Y){
  static enum{X_ON,X_OFF,xD} m1=X_ON;
  static enum{Y_ON,Y_OFF,yD} m2=Y_ON;
   if((m1==xD)||(m2==yD)){
    return;
   }
   if( (abs(error_X) <= 1651) && (abs(error_X) > 100) ){
    m1=X_OFF;
   }
  if( (abs(error_X) <= 1651) && (abs(error_X) > 100) ){
    m2=Y_OFF;
   }
   if((m1==X_ON)&&(m2==Y_ON)){
      analogWrite(E_X, constrain(motorSignal_X, 50,255)); 
      analogWrite(E_Y, constrain(motorSignal_Y, 50,255));
      return; 
   }else if((m1==X_ON)){
      analogWrite(E_X, constrain(motorSignal_X, 50,255)); 
      analogWrite(E_Y, 0);
      return;
   } else if(m2==Y_ON){
      analogWrite(E_X, 0);
      analogWrite(E_Y, constrain(motorSignal_Y, 50,255));
   } else if((m1==X_OFF)&&(m2==Y_OFF)){
      for(int i = ((abs(motorSignal_Y)+abs(motorSignal_X))/2); i>50; i-- ){
        analogWrite(E_X, i);
        analogWrite(E_Y, i);
        if( (abs(error_X) <= 1651) && (abs(error_X) > 100) ){
          m1=xD;
        }
        if( (abs(error_X) <= 1651) && (abs(error_X) > 100) ){
          m2=yD;
        }
         if((m1==xD)&&(m2==yD)){
          return;
         }
       }
   } else{
    return;
   }
  
}
void get_m_target_count(float targetDistance, float MOTOR_CONST, float &target_count){
  target_count = MOTOR_CONST*targetDistance;
}
void loop_motor_control(float &motorSignal, int M_A, int E_A, const float MOTOR_A_CONST,float &error, volatile int &count){
  if (motorSignal >0){
    digitalWrite(M_A, HIGH);
    Serial.print("Current Distance: ");
    Serial.println(count/MOTOR_A_CONST);
    // Serial.println(motorEncoderToMotorDistance(count_X));
  } else if (motorSignal < 0 ){
    digitalWrite(M_A, LOW);
    Serial.print("Current Distance: ");
    Serial.println(count/MOTOR_A_CONST);
    //  Serial.println(motorEncoderToMotorDistance(count_X));

    motorSignal = -motorSignal;

    // Serial.println("Motor Dir: LOW ");
    // Serial.print("Motor Signal: ");
    // Serial.println(motorSignal);
  } else {
    motorSignal = 0;
    analogWrite(E_A, 0);
    //kill motor
    //die bad day was here <------------------[*;::;*]
  }
}
///// ---------------------------------------[START]
void kp_operation(float targetDistance_y_left,float targetDistance_x_right, float Kp_Y, float kp_X){
float targetCounts_X;
float targetCounts_Y;
float motorSignal_X;
float motorSignal_Y;
float error_Y;
float error_X;
float average_count = (count_X+count_Y)/2.0;
float avaerage_const =(MOTOR_RIGHT_CONST_X+MOTOR_LEFT_CONST_Y)/2.0;
float targetDistance =(targetDistance_y_left+targetDistance_x_right)/2.0;
get_m_target_count(targetDistance_x_right,MOTOR_RIGHT_CONST_X,targetCounts_X);
get_m_target_count(targetDistance_y_left,MOTOR_LEFT_CONST_Y,targetCounts_Y);
do{
  //PRINT BLOCK ->]
  Serial.print("Target Counts_X: ");
  Serial.println(targetCounts_X);
  Serial.print("Target Counts_Y: ");
  Serial.println(targetCounts_Y);
  Serial.print("Current Count_X: ");
  Serial.println(count_X);
  Serial.print("Current Count_Y: ");
  Serial.println(count_Y);
  Serial.print("Error_X: ");
  Serial.println(error_X);
  Serial.print("Error_Y: ");
  Serial.println(error_Y);
 //[<-
  error_X = targetCounts_X - count_X;
  error_Y = targetCounts_Y - count_Y;
  motorSignal_X = kp_X*error_X;
  motorSignal_Y = Kp_Y*error_Y;
  //X-y_LOOP -->]
  loop_motor_control(motorSignal_X, M_X, E_X, MOTOR_RIGHT_CONST_X, error_X, count_X);
  loop_motor_control(motorSignal_Y, M_Y, E_Y, MOTOR_LEFT_CONST_Y, error_Y, count_Y);

  motor_power_ramp_down(motorSignal_X,motorSignal_Y,error_X, error_Y, E_X,E_Y);
  
  }while(  !( (abs(count_X/MOTOR_RIGHT_CONST_X - targetDistance_x_right) <= 1) && (abs(count_Y/MOTOR_LEFT_CONST_Y - targetDistance_y_left) <= 1) ) ); //motorEncoderToMotorDistance(count_X)
  
  analogWrite(E_X, 0);
  analogWrite(E_Y, 0);
  die_badday();
  Serial.println("Slay");
  }
///// ---------------------------------------[END]
  
  
void pContorlRightMotor(float targetDistance, float Kp){
float targetCounts;
get_m_target_count(targetDistance,MOTOR_RIGHT_CONST_X,targetCounts);
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
    Serial.print("Current Distance: ");
    Serial.println(count_X/65.62);
    // Serial.println(motorEncoderToMotorDistance(count_X));
    

    if( (abs(error) <= 1651) && (abs(error) > 100) ){
      for(int i = abs(motorSignal); i>50; i-- ){
        analogWrite(E_X, i);

        if ( (abs(error) >= 1651) && (abs(error) < 100)){
          break;
        }
      }

      
    } else{
      analogWrite(E_X, constrain(motorSignal, 50,255)); 
    }

  } else if (motorSignal < 0 ){
    digitalWrite(M_X, LOW);
    Serial.print("Current Distance: ");
    Serial.println(count_X/65.62);
    //  Serial.println(motorEncoderToMotorDistance(count_X));

    motorSignal = -motorSignal;

    // Serial.println("Motor Dir: LOW ");
    // Serial.print("Motor Signal: ");
    // Serial.println(motorSignal);
    if( (abs(error) <= 1651) && (abs(error) > 100) ){
      for(int i = abs(motorSignal); i>50; i-- ){
        analogWrite(E_X, i);

        if ( (abs(error) >= 1651) && (abs(error) < 100)){
          break;
        }
      }
      
    } else{
      analogWrite(E_X, constrain(motorSignal, 50,255)); 
    }

  } else {
    motorSignal = 0;
    analogWrite(E_X, 0);
    die_badday();
    

  }


}while(abs(count_X/65.62 - targetDistance)>= 1); //motorEncoderToMotorDistance(count_X)
 
analogWrite(E_X, 0);
die_badday();
Serial.println("Slay");


}

void pContorlLeftMotor(float targetDistance, float Kp){
  float jonathon = 63.7;

// float targetCounts = distanceToMotorEncoder(targetDistance);
float targetCounts = jonathon*targetDistance;
Serial.print("Target Counts: ");
Serial.println(targetCounts);


float motorSignal;
float error;

do{

  Serial.print("Target Counts: ");
  Serial.println(targetCounts);

  error = targetCounts - count_Y;

  Serial.print("Current Count: ");
  Serial.println(count_Y);


  Serial.print("Error: ");
   Serial.println(error);

  motorSignal = Kp*error;

  if (motorSignal >0){
    digitalWrite(M_Y, HIGH);
    Serial.print("Current Distance: ");
    Serial.println(count_Y/jonathon);
    // Serial.println(motorEncoderToMotorDistance(count_X));
    

    if( (abs(error) <= 1651) && (abs(error) > 100) ){
      for(int i = abs(motorSignal); i>50; i-- ){
        analogWrite(E_Y, i);

        if ( (abs(error) >= 1651) && (abs(error) < 100)){
          break;
        }
      }

      
    } else{
      analogWrite(E_Y, constrain(motorSignal, 50,255)); 
    }

  } else if (motorSignal < 0 ){
    digitalWrite(M_Y, LOW);
    Serial.print("Current Distance: ");
    Serial.println(count_Y/jonathon);
    //  Serial.println(motorEncoderToMotorDistance(count_X));

    motorSignal = -motorSignal;

    // Serial.println("Motor Dir: LOW ");
    // Serial.print("Motor Signal: ");
    // Serial.println(motorSignal);
    if( (abs(error) <= 1651) && (abs(error) > 100) ){
      for(int i = abs(motorSignal); i>50; i-- ){
        analogWrite(E_Y, i);

        if ( (abs(error) >= 1651) && (abs(error) < 100)){
          break;
        }
      }
      
    } else{
      analogWrite(E_Y, constrain(motorSignal, 50,255)); 
    }

  } else {
    motorSignal = 0;
    analogWrite(E_Y, 0);
    die_badday();
    

  }


}while(abs(count_Y/jonathon - targetDistance)>= 1); //motorEncoderToMotorDistance(count_X)
 
analogWrite(E_Y, 0);
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
    kp_operation(50,-50,2,2);
    // pContorlRightMotor(-50, 2);
    // pContorlLeftMotor(-50, 2);

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
    count_Y++;
  } else {
    count_Y--;
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

  int distMot1 = motorEncoderToMotorDistance(count_Y);
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