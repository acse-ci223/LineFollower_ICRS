// Include libraries
#include "Servo.h" 
#include "PID_v1.h"

// Pin definitions
#define lMotorPin1 10  // Left motor pins
#define lMotorPin2 11
#define rMotorPin1 5  // Right motor pins
#define rMotorPin2 6
#define servoPin 3 // Servo pin
#define ir1Pin A0 // Line sensor pin 1
#define ir2Pin A1 // Line sensor pin 2
#define ir3Pin A2 // Line sensor pin 2
#define statusPin 13

// Define constants
double servoOffset = 30; //offset for middle position

// Variables 
int lMotorSpeed;
int rMotorSpeed;
int servoAngle;
int servoRangeL = -30;
int servoRangeR = 30;
double Setpoint = 1; //target value for line center

//Define Variables we'll be connecting to
double Input, Output;

//Define the aggressive and conservative Tuning Parameters
// double aggKp=4, aggKi=0.2, aggKd=1;
// double consKp=1, consKi=0.05, consKd=0.25;

double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Servo steeringServo;

float w1 = 1;
float w2 = 0.01;
float w3 = -1;

float ir1Val, ir2Val, ir3Val;

void setup() {

  Serial.begin(9600);

  steeringServo.attach(servoPin);  
  // steeringServo.write(90 + servoOffset);

  pinMode(lMotorPin1, OUTPUT);
  pinMode(lMotorPin2, OUTPUT); 
  pinMode(rMotorPin1, OUTPUT);
  pinMode(rMotorPin2, OUTPUT);

  pinMode(statusPin, OUTPUT);

  pinMode(ir1Pin, INPUT);
  pinMode(ir2Pin, INPUT); 
  pinMode(ir3Pin, INPUT); 

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10); 
  myPID.SetOutputLimits(-30, 30);

  float delayTime = 5.0;

  bool leftright = false;
  steeringServo.write(20);

  for(int i=0; i< delayTime*10; i++){
    if(i%10 == 0){
      digitalWrite(statusPin, 1);
      leftright!=leftright;
    }else{
      digitalWrite(statusPin, 0);
    }
    steeringServo.write(45);

    delay(100);
  }
}

void loop() {
  digitalWrite(statusPin, 1);

  ir1Val = 1.0-analogRead(ir1Pin)/1024.0;  
  ir2Val = 1.0-analogRead(ir2Pin)/1024.0;
  ir3Val = 1.0-analogRead(ir3Pin)/1024.0;
  
  // Print the sensor values  
  Serial.print("Left: "); 
  Serial.print(ir1Val);

  Serial.print(" | Center: ");
  Serial.print(ir2Val);  

  Serial.print(" | Right: ");
  Serial.print(ir3Val); 

  // Determine position using all sensors
  Input = (w1*ir1Val + w2*ir2Val + w3*ir3Val) / 1;
  Serial.print(" | Input: ");
  Serial.print(Input);

  myPID.Compute();  // Calculate new servo angle

  Serial.print(" | Output: ");
  Serial.print(Output);

  servoAngle = 0 - (Output);
  steeringServo.write(servoAngle);
  Serial.print(" | Angle: ");
  Serial.print(servoAngle);


  // Convert angle to motor speeds using some math
  // lMotorSpeed = 200 - servoAngle; 
  // rMotorSpeed = 200 + servoAngle;
  lMotorSpeed = 200; 
  rMotorSpeed = 200;
  
  // Drive motors

  // analogWrite(lMotorPin1, 0); 
  // analogWrite(lMotorPin2, lMotorSpeed);
  // analogWrite(rMotorPin2, rMotorSpeed);
  // analogWrite(rMotorPin1, 0);

  Serial.println();

  delay(100);
}