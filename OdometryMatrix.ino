#include <BasicLinearAlgebra.h>;

#define rightMotor 1;
#define leftMotor 2;
#define rightIR 3;
#define leftIR 4;

#define wheelRadius 2;
#define baseline 10;

float circumference = wheelRadius * TWO_PI;
float tickDistance = wheelRadius * deg2rad(90/75);
float x = 0, xDelta, y = 0, yDelta, thetaZ = 0, deltaThetaZ;
float thetaZGlobal = tickDistance / baseline;
float xGlobal = (baseline / 2_ * sin(thetaZGlobal);
float yGlobal = (baseline / 2) * (1 - cos(thetaZGlobal));

Matrix<4,4> rotationRight = {cos(thetaZGlobal), -sin(thetaZGlobal), 0, 0,
                             sin(thetaZGlobal), cos(thetaZGlobal),  0, baseline/2
                             0,                 0,                  1, 0,
                             0,                 0,                  0, 1};
Matrix<4,4> rotationLeft = {cos(thetaZGlobal), -sin(thetaZGlobal), 0, 0,
                             sin(thetaZGlobal), cos(thetaZGlobal),  0, baseline/2
                             0,                 0,                  1, 0,
                             0,                 0,                  0, 1};
Matrix<4,4> translationRight = {1, 0, 0, 0,
                                0, 1, 0, -baseline
                                0, 0, 1, 0,
                                0, 0, 0, 1};
                                
Matrix<4,4> translationLeft =  {1, 0, 0, 0,
                                0, 1, 0, -baseline
                                0, 0, 1, 0,
                                0, 0, 0, 1};

Matrix<4,4> transformRight = rotationRight * translationRight;
Matrix<4,4> transfromLeft = rotationLeft * translationLeft;

Matrix<4,4> global = {1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1};


void setup() {
  Serial.begin(9600);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftIR), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightIR), rightTick, CHANGE);  
}

void loop() { }

void leftTick(){
  global = global * transformLeft;
  Serial.print("Theta: " + global(0,0) + "   X: " + global(1,3) + "   Y: " + global(0,3)):
}

void rightTick(){
  global = global * transformRight;
  Serial.print("Theta: " + global(0,0) + "   X: " + global(1,3) + "   Y: " + global(0,3)):
}
