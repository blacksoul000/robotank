#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x4

const int8_t boardL1 = 8;
const int8_t boardL2 = 7;
const int8_t boardPwmL = 9;
const int8_t boardR1 = 5;
const int8_t boardR2 = 4;
const int8_t boardPwmR = 6;

const int16_t maxDeviation = 32767;

struct RpiPkg
{
    int8_t bySpeed : 1;
    int8_t deviceId : 4;
    int8_t reserve : 3;
    int16_t x = 0;
    int16_t y = 0;
};

Servo gun;
Servo camera;

void setup() {
  Serial.begin(57600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
//  Wire.setClock(400000L);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  // Engine Left
  pinMode(boardL1, OUTPUT);
  pinMode(boardL2, OUTPUT);
  pinMode(boardPwmL, OUTPUT);

  // Engine Right
  pinMode(boardR1, OUTPUT);
  pinMode(boardR2, OUTPUT);
  pinMode(boardPwmR, OUTPUT);
  
  // Servo
//  gun.attach(A3);
//  camera.attach(A2);

  Serial.println("Ready!");
}

void loop() {
}

void applySpeed(int16_t speed, int8_t pin1, int8_t pin2, int8_t pinPwm)
{
  int16_t absSpeed = abs(speed);
  Serial.print(speed);
  Serial.print(" | ");
  Serial.println(pinPwm);
  analogWrite(pinPwm, absSpeed > 250 ? 250 : absSpeed);
  if (speed > 0)
  {
    digitalWrite(pin2, LOW);
    digitalWrite(pin1, HIGH);
  }
  else
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
}

// callback for received data
void receiveData(int byteCount) 
{
  if (byteCount == sizeof(RpiPkg))
  {
    processRpiData();
  }
  else
  {
    // Garbage? Can't handle, drop.
    while(Wire.available()) Wire.read();
  }
}

void processRpiData()
{
  char data[sizeof(RpiPkg)];
  int i = 0;
  while(Wire.available()) {
    data[i] = (char)Wire.read();
    ++i;
  }

  RpiPkg pkg = *reinterpret_cast< const RpiPkg* >(data);
  if (pkg.deviceId != 0) return; // chassis
  
  float speed = 1.0 * pkg.y / maxDeviation * 255;
  float turnSpeed = 1.0 * pkg.x / maxDeviation * 255;

  applySpeed(speed + turnSpeed, boardL1, boardL2, boardPwmL);
  applySpeed(speed - turnSpeed, boardR1, boardR2, boardPwmR);
//  int y = 1.0 * pkg.y / maxDeviation * 255;
//  applySpeed(y, boardL1, boardL2, boardPwmL);
//  applySpeed(y, boardR1, boardR2, boardPwmR);

//  Serial.print(pkg.bySpeed);
//  Serial.print(" | did=");
//  Serial.print(pkg.deviceId);
//  Serial.print(" | x=");
//  Serial.print(pkg.signX ? "-" : "");
//  Serial.print(pkg.x);
//  Serial.print(" | y=");
//  Serial.print(pkg.signY ? "-" : "");
//  Serial.println(pkg.y);
}

// callback for sending data
void sendData() 
{
  struct ArduinoPkg
  {
      int16_t x = 0;
      int16_t y = 0;
  } pkg;

  pkg.x = 65.34;
  pkg.y = 11.11;//camera.read();
  Wire.write(reinterpret_cast< const unsigned char* >(&pkg), sizeof(pkg));
}

