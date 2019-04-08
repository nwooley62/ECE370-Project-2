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

void setup() {
  Serial.begin(9600);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftIR), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightIR), rightTick, CHANGE);  
}

void loop() { }

void leftTick(){
  thetaZ += thetaZGlobal;
  yDelta = xGlobal*cos(thetaZGlobal) + yGlobal*sin(thetaZGlobal);
  xDelta = xGlobal*sin(thetaZGlobal) + yGlobal*cos(thetaZGlobal);
  y -= yDelta;
  x += xDelta;
  //Serial.println("X: " + x + "  Y: " + y);
}

void rightTick(){
  thetaZ -= deltaThetaZ;
  yDelta = xGlobal*cos(thetaZGlobal) + yGlobal*sin(thetaZGlobal);
  xDelta = xGlobal*sin(thetaZGlobal) + yGlobal*cos(thetaZGlobal);
  y += yDelta;
  x += xDelta;
  //Serial.println("X: " + x + "  Y: " + y);
}
