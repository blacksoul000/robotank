#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#define SLAVE_ADDRESS 0x4

// chassis engines
const int8_t boardL1 = 8;
const int8_t boardL2 = 7;
const int8_t boardPwmL = 9;
const int8_t boardR1 = 5;
const int8_t boardR2 = 4;
const int8_t boardPwmR = 6;

// tower mpu
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

const int16_t maxDeviation = 32767;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
  mpuInterrupt = true;
}

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
MPU6050 mpu;

uint32_t ms, ms1 = 0;

String prefix;
String buf;
bool waitPrefix = true;

void setup() {
  Serial.begin(115200); // start serial for output

  prefix += char(0x55);
  prefix += char(0x55);

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

  // initialize i2c as slave
  Serial.println("Initializing I2C...");
//  Wire.setClock(400000L);

  mpu.initialize();
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-4188);
  mpu.setYGyroOffset(-865);
  mpu.setZGyroOffset(2853);
  mpu.setXAccelOffset(65);
  mpu.setYAccelOffset(-29);
  mpu.setZAccelOffset(-31);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println("Ready!");
}

void loop()
{
  ms = millis();
  if ((ms - ms1) > 1000 || ms < ms1 )
  {
       ms1 = ms;
       sendData();
  }
//  processAccelGyro();
//  delay(50);
}

void processAccelGyro()
{

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
//    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180/M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180/M_PI);
    
//    float mpuYaw  = ypr[0] * 180 / M_PI;
//    float mpuPitch = ypr[1] * 180 / M_PI;
//    float mpuRoll = ypr[2] * 180 / M_PI;
  }
}  // processAccelGyro()

void applySpeed(int16_t speed, int8_t pin1, int8_t pin2, int8_t pinPwm)
{
  int16_t absSpeed = abs(speed);
//  Serial.print(speed);
//  Serial.print(" | ");
//  Serial.println(pinPwm);
  analogWrite(pinPwm, absSpeed > 255 ? 255 : absSpeed);
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

void sendData()
{
  struct ArduinoPkg
  {
    int16_t x = 0;
    int16_t y = 0;
  } pkg;

  pkg.x = ypr[0] * 180 / M_PI / (360.0 / 32768); //* 180 / M_PI / (360 / 32768);
  pkg.y = ypr[1] * 180 / M_PI / (360.0 / 32768); //* 180 / M_PI / (360 / 32768);
//  pkg.y = camera.read() / (360.0 / 32768);

  Serial.write(prefix.c_str(), prefix.length());
  Serial.write(reinterpret_cast< const unsigned char* >(&pkg), sizeof(pkg));
}

void serialEvent()
{
  while (Serial.available())
  {
    buf += (char)Serial.read();
  }
  
  if (waitPrefix)
  {
    if (buf.length() < prefix.length()) return;
    
    int8_t pos = buf.indexOf(prefix);
    if (pos == -1)
    {
      buf.remove(0, buf.length() - prefix.length());
      return;
    }
    else
    {
      buf.remove(0, pos);
      waitPrefix = false;
    }
  }
  if (!waitPrefix)
  {
    const uint8_t packetSize = prefix.length() + sizeof(RpiPkg);
    if (buf.length() < packetSize) return;
    
    RpiPkg pkg = *reinterpret_cast< const RpiPkg* >(buf.c_str() + prefix.length());
    waitPrefix = true;
    buf.remove(0, packetSize);
    if (pkg.deviceId != 0) return; // chassis
  
    float speed = 1.0 * pkg.y / maxDeviation * 255;
    float turnSpeed = 1.0 * pkg.x / maxDeviation * 255;
  
    applySpeed(speed + turnSpeed, boardL1, boardL2, boardPwmL);
    applySpeed(speed - turnSpeed, boardR1, boardR2, boardPwmR);

  //  int y = 1.0 * pkg.y / maxDeviation * 255;
  //  applySpeed(y, boardL1, boardL2, boardPwmL);
  //  applySpeed(y, boardR1, boardR2, boardPwmR);
//    Serial.println("");
//    Serial.print(pkg.bySpeed);
//    Serial.print(" | did=");
//    Serial.print(pkg.deviceId);
//    Serial.print(" | x=");
//    Serial.print(pkg.x);
//    Serial.print(" | y=");
//    Serial.println(pkg.y);
  }
}
