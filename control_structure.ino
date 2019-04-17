#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 

#define leftIR 1
#define rightIR 2
#define leftMotor 3
#define rightMotor 4
#define baseline 10
#define radius 25

char ssir[] = SECREC_SSID;
char pass[] = SECRET_PASS;
int status = LW_IDLE_STATUS;
IPAddress ip(192, 168, 1, 1);
WiFiClient client;

float circumference = radius * TWO_PI;
float tickDistance = radius * deg2rad( 90 / 75.8 );
float thetaZ = 0, _thetaZ = tickDistance / baseline;
float x = 0, _x = ( BASELINE / 2 ) * sin ( _thetaZ ), _x0;
float y = 0, _y = ( BASELINE / 2 ) * ( 1 - cos ( _thetaZ ) ), _y0;
int a, ax, ay, az, ML, MR;
int[] temp;
double velocity, theta;

struct control{
  double velocity;
  double theta;
  int mode;
}


void setup() {
  //AP Setup
  //setup microcontroller to connect to an AP
  //Initialize serial and wait for port to open:
  WiFi.setPins(8,7,4,2);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
 // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");

  //Open Port
  openPort();
  
  //Initialize Structure
  initStruct();
  
  //Set IR and Motor Pins
  pinMode(leftIR, INPUT_PULLUP);
  pinMode(rightIR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftIR, checkLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIR, checkRight, RISING);

  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);

  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);
  
  //IMU Setup
  IMUSetup(1);

}

void loop() {
  //Check UDP (Non Blocking)
  a = checkUDP();
  //Parse UDP

  //velocity, theta = parse(a)
  
  //Set Speed
  setVelocity(velocity);
  //Set Direction
  setAngle(theta);
  //Set MotorLeft and MotorRight (global variables)

  //ML, MR = parse(angle, velocity)
  
  //If(picked up) motorLeft && motorRight = 0
  //if(pickedUp()){
    setVelocity(0);
  //}
}

void checkLeft(){
  thetaZ += _thetaZ;
  _x0 = ( ( _x * cos( _thetaZ ) ) + ( _y * sin( _thetaZ ) ) );
  _y0 = ( ( _x * sin( _thetaZ ) ) + ( _y * cos( _thetaZ ) ) );
  y -= _y0;
  x += _x0;
}

void checkRight(){
  thetaZ -= _thetaZ;
  _x0 = ( ( _x * cos( _thetaZ ) ) + ( _y * sin( _thetaZ ) ) );
  _y0 = ( ( _x * sin( _thetaZ ) ) + ( _y * cos( _thetaZ ) ) );
  y += _y0;
  x += _x0;
}
void openPort(){
  //open port of arduino for pi input
}
void IMUSetup(int sensitivity){
  //set up IMO to take info from it
}
void initStruct(){
  //set up motor values, buffer input and odometry values
}
int checkIMU(){
  //check IMU for updates
}
int[] getIMU(){
  //return IMU sensor values
}
int checkUDP(){
  //check if UDP message has been sent
}
void setVelocity(double velocity){
  
}
void setAngle(double angle){
  
}
void pickedUp(){
  bool pickedUp = false;
  //if(IMU.Z > threshold){
  pickedUP = true;
  //}

  return pickeUp;
