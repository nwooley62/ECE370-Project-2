#include <WiFi101.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <LSM303.h>
#include <Wire.h>
#include "arduino_secrets.h" 

#define BASELINE 90
#define RADIUS 25
#define leftIR 6
#define rightIR 12
#define rightMotorA 5
#define rightMotorB 9
#define leftMotorA 10
#define leftMotorB 11
#define gearRatio 0.013333 //(1/75.8)

//ODOMETRY CONSTANTS
float circBase = 2*PI*(BASELINE);
float circWheel = 2*PI*(RADIUS);
float tickDistance = circWheel/(75.8*2);
//float tickRadians = tickDistance/ BASELINE;
float phiTick = (PI*tickDistance)/(circBase);
float deltaX = (BASELINE/2)*sin(phiTick);
float deltaY = (BASELINE/2) - (BASELINE/2)*cos(phiTick);

//WIFI CONNECTION
char ssid[] = SECRET_SSID;        
char pass[] = SECRET_PASS;    
int status = WL_IDLE_STATUS; 

//UDP OBJECTS
unsigned int localPort = 5005; 
WiFiUDP Udp;

//PACKET BUFFERS
char packetBuffer[1024]; 
char recieveBuffer[1024];

//IMU VARIABLES
LSM303 compass;
float IMUheading;

//GLOBAL VARIABLES 
float xGlobal, yGlobal, thetaGlobalRad, thetaGlobalDeg;
float velocity = 0; 

//MOTOR CONTROL
bool pickup = false;
int motorRight = 0, motorLeft = 0;
int robotMode = 0;

//KP LOOPS
float thetaLeft = 0, thetaRight = 0, phiDirection = 0;
float errorLeft = 0, errorRight = 0, errorDirection = 0;
float velocityLeft = 0, velocityRight = 0, angularSpeed = 0;
float KpSpeed = 5, KpDirection = 6;
int countRight, countLeft = 0;
float phiDesired = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct returnPacket{      //Packet to be sent to Pi
  double x;
  double y;
  double theta;
} returnPacket;
typedef struct cmdPacket{         //Packet to be recieved from Pi
  double velocity;
  double phi;
  int mode;
} cmdPacket;

returnPacket rPacket;
cmdPacket cPacket;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  analogWrite(rightMotorA, 0);
  analogWrite(rightMotorB, 0);
  analogWrite(leftMotorA, 0);
  analogWrite(leftMotorB, 0);
  
  WiFi.setPins(8,7,4,2);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  setupWifi();
  openPort();
  motorSetup();
  IRsetup();
  IMUsetup();
  velocity = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
 readPacket();
 //checkIMU();
 //pickupBot();
 setMotor();
 if(robotMode == 1){          //Manual Drive for odometry test
  motorLeft = velocity;
  motorRight = velocity;
 }else if(robotMode == 2){    //Compass code
  //setDirection(phiDesired);
  motorLeft = 75;
  motorRight = 0;
 }else if(robotMode == 3){
  motorLeft = 0;
  motorRight = 0;
 }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupWifi(){                                                       
   while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to Wifi");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void openPort(){
  Udp.begin(localPort);
  Serial.println("Port Open");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void motorSetup(){
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRsetup(){
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftIR), doTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIR), doTickRight, RISING);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMUsetup(){
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-1153, +2034, -3981};
  compass.m_max = (LSM303::vector<int16_t>){+3754, +5738, +672};
  Serial.println("IMU Setup");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void readPacket(){
  //Serial.print("reading packet.  PacketSize: ");
  int packetSize = Udp.parsePacket();
  //Serial.println(packetSize);
  //int len = Udp.read(recieveBuffer, 255);
  if(packetSize){
    
    //Serial.println("Detected Packet");
    int len = Udp.read(recieveBuffer, 1024);
    if(len > 0) recieveBuffer[len] = 0;
    Serial.println("Packet says: ");
    Serial.print(recieveBuffer);                      //If running into UDP issues, the recieve buffer prints as blank
    memcpy(&cPacket, recieveBuffer, sizeof(cmdPacket));
    /*
     * mode 0 = return x,y,phi
     * mode 1 = driving command
     * mode 2 = cardinal command
     * mode 3 = stop
     * mode 4 = restart
     */

     if(cPacket.mode == 0){
      Serial.println("Mode 0");
      Serial.println(cPacket.velocity);
      Serial.println(cPacket.phi);
      writePacket();
      robotMode = 0;
      
     }else if(cPacket.mode == 1){
      Serial.println("Mode 1");
      velocity = cPacket.velocity;
      phiDesired = cPacket.phi;
      robotMode = 1;
      
     }else if(cPacket.mode == 2){
      Serial.println("Mode 2");
      //setDirection(cPacket.phi);
      phiDesired = cPacket.phi;
      Serial.print("setDirection to: ");
      Serial.print(cPacket.phi);
      robotMode = 2;
      
     }else if(cPacket.mode == 3){
      Serial.println("Mode 3");
      velocity = 0;
      phiDesired = thetaGlobalDeg;
      pickup = false;
      robotMode = 3;
      
     }else if(cPacket.mode == 4){
      Serial.println("Mode 4");
      robotMode = 4;
      xGlobal = 0;
      yGlobal = 0;
      thetaGlobalRad = 0;
      thetaGlobalDeg = 0;
     }

  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void writePacket(){
  rPacket.x = xGlobal;
  rPacket.y = yGlobal;
  rPacket.theta = thetaGlobalDeg;

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  char transmitBuffer[1024] = {0};
  memcpy(transmitBuffer, &rPacket, sizeof(returnPacket));
  Udp.write(transmitBuffer, sizeof(returnPacket));
  Udp.endPacket();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pickupBot(){
  //set motor speed to 0
  //set bool picup to true
  int threshold = 22000;
  if(compass.a.z > threshold){
    pickup = true;
    Serial.println("Picked Up");
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkIMU(){
  compass.read();
  IMUheading = compass.heading((LSM303::vector<int>){-1, 0, 0});
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void doTickLeft(){
    countLeft++;
    thetaGlobalRad -= phiTick;
    thetaGlobalDeg = thetaGlobalRad * (180/PI);

    xGlobal += deltaX*cos(thetaGlobalRad) + deltaY*sin(thetaGlobalRad);
    yGlobal += deltaX*sin(thetaGlobalRad) + deltaY*cos(thetaGlobalRad);
    
    Serial.print("X: ");
    Serial.print(xGlobal);
    Serial.print("  Y: ");
    Serial.print(yGlobal);
    Serial.print("  Theta: ");
    Serial.println(thetaGlobalDeg);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void doTickRight(){
    countRight++;
    thetaGlobalRad += phiTick;
    thetaGlobalDeg = thetaGlobalRad * (180 / PI);

    xGlobal += deltaX*cos(thetaGlobalRad) + deltaY*sin(thetaGlobalRad);
    yGlobal += deltaX*sin(thetaGlobalRad) + deltaY*cos(thetaGlobalRad);

    Serial.print("X: ");
    Serial.print(xGlobal);
    Serial.print("  Y: ");
    Serial.print(yGlobal);
    Serial.print("  Theta: ");
    Serial.println(thetaGlobalDeg);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool turnLeft;
void setMotor(){
//if(robotMode == 2){
//  if(!turnLeft){
//    motorRight = 2*velocityRight + (abs(angularSpeed)*BASELINE)/(2*RADIUS);
//   motorLeft = 0;
//  }else{
//    motorLeft = 2*velocityLeft + (abs(angularSpeed)*BASELINE)/(2*RADIUS);
//    motorRight = 0;
//  }
//}
  if(motorLeft > 100){
    motorLeft = 100;
  }else if(motorLeft < 0){
    motorLeft = 0;
  }

  if(motorRight > 100){
    motorRight = 100;
  }else if(motorRight < 0){
    motorRight = 0;
  }
  if(pickup){
    motorLeft = 0;
    motorRight = 0;
  }

  analogWrite(rightMotorA, motorRight);
  analogWrite(rightMotorB, 0);
  analogWrite(leftMotorA, motorLeft);
  analogWrite(leftMotorB, 0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setMotorLeft(float x){
  thetaLeft = float(countLeft) * 0.5 * gearRatio;
  errorLeft = x - thetaLeft;
  velocityLeft = errorLeft * KpSpeed;
  countLeft = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setMotorRight(float x){
  thetaRight = float(countRight) * 0.5 * gearRatio;
  errorRight = x - thetaRight;
  velocityRight = errorRight * KpSpeed;
  countRight = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setDirection(float x){

  //phiDirection = IMUheading;
  phiDirection = thetaGlobalDeg;
  errorDirection = x - phiDirection;
  if(x == 0){
    if(phiDirection >= 0 && phiDirection < 30){
      turnLeft = true;
    }else{
      turnLeft = false;
    }
  }else{
    if(errorDirection >= 0){
      turnLeft = true;
    }else{
      turnLeft = false;
    }
  }
  angularSpeed = errorDirection * KpDirection;
  
}
