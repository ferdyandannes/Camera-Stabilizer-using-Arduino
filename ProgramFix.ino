#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13
#define buttonPinn 7
#define control 2
#define buttonPin 5
#define ledPin 3
bool blinkState = true;

Servo ServoPitch;
Servo ServoRoll;
Servo ServoYaw;

int pos = 0;

int ServoPitchPos = 0;
int ServoRollPos = 0;
int ServoYawPos = 0;

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;

//PROGRAM JOYSTICK
int xPin = A0;
int yPin = A1;
int xPosition;
int yPosition;
int buttonPos = 0;
int xPos, yPos;

//PILIH MODE
int val = 0;

//COUNTER MODE
int buttonPushCounter = 0;
int buttonState = 1;
int lastbuttonState = 1;

//PROGRAM INTERFACE LED
int ledState = LOW;
unsigned long previousMillis = 0;
unsigned long intervalauto = 1000;
unsigned long intervalmanual = 500;
unsigned long currentMillis;

float x1, x2, x3, c1, c2, c3, c3baru;

MPU6050 mpu;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];

#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

void setup()
{
  ServoPitch.attach(10);  // attaches the servo on D11 to the servo object
  ServoRoll.attach(11);  // Second servo on D11
  ServoYaw.attach(9); //Third servo on D9
  pinMode(buttonPinn, INPUT_PULLUP);
  pinMode(control, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(control, INPUT);
  delay(50);
  ServoPitch.write(100);  // These are command checks to see if the servos work and
  ServoRoll.write(100);  // to help w/ the initial installation.
  ServoYaw.write(100);
  delay(500);   // Make sure these movements are clear from the rest of the chassis.
  ServoPitch.write(90);
  ServoRoll.write(90);
  ServoYaw.write(90);
  delay(500);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);
  while (!Serial);      // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(120);
  mpu.setYGyroOffset(25);
  mpu.setZGyroOffset(-59);
  mpu.setXAccelOffset(488);
  mpu.setYAccelOffset(-3119);
  mpu.setZAccelOffset(604);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }
  pinMode(LED_PIN, OUTPUT);
}

void loop(void)
{
  currentMillis = millis();
  val = digitalRead(control);
  if(val==LOW){
  processAccelGyro();
  if (currentMillis - previousMillis >= intervalauto) 
  {
    previousMillis = currentMillis;
    if (ledState == LOW) 
    {
      ledState = HIGH;
    } 
    else 
    {
      ledState = LOW;
    }
   digitalWrite(ledPin, ledState);
  }
  }
  
  else if(val == HIGH)
  {
    buttonState = digitalRead(buttonPinn);
    if(buttonState != lastbuttonState)
    {
      if (buttonState == LOW) 
      {
        if(buttonPushCounter == 2)
        {
          buttonPushCounter = 0;
        }
        buttonPushCounter++;
        Serial.println("on");
        Serial.print("number of button pushes: ");
        Serial.println(buttonPushCounter);
        delay(200);
      } 
      delay(300);
    }
    if (buttonPushCounter == 1) 
    {
      xPosition = analogRead(xPin);
      if(600 <= xPosition >= 800)
      {
        xPos+=3;
        if(xPos == 180)
        {
          xPos = 180;
        }
      }
      else if(800 < xPosition <= 1024)
      {
        xPos+=5;
        if(xPos == 180)
        {
          xPos = 180;
        }
      }
      else if(200 <= xPosition <= 400)
      {
        xPos-=3;
        if(xPos == 0)
        {
          xPos = 0;
        }
      }
      else if(0 <= xPosition < 200)
      {
        xPos-=5;
        if(xPos == 0)
        {
          xPos = 0;
        }
      }
      ServoYaw.write(xPos);

      yPosition = analogRead(yPin);
      if(600 <= yPosition >= 800)
      {
        yPos+=3;
        if(yPos == 180)
        {
          yPos = 180;
        }
      }
      else if(800 < yPosition <= 1024)
      {
        yPos+=5;
        if(yPos == 180)
        {
          yPos = 180;
        }
      }
      else if(200 <= yPosition <= 400)
      {
        yPos-=3;
        if(yPos == 0)
        {
          yPos = 0;
        }
      }
      else if(0 <= yPosition < 200)
      {
        yPos-=5;
        if(yPos == 0)
        {
          yPos = 0;
        }
      }
      ServoPitch.write(yPos);
    
      if (currentMillis - previousMillis >= intervalmanual) 
      {
        previousMillis = currentMillis;
        if (ledState == LOW) 
        {
          ledState = HIGH;
        } 
        else 
        {
          ledState = LOW;
        }
        digitalWrite(ledPin, ledState);
      }
      delay(15);
    }
    else if (buttonPushCounter == 2) 
    {
      ServoYaw.write(30);
      for (pos = 30; pos <= 150; pos += 1) 
      { 
        ServoYaw.write(pos);
        if(pos == 150)
        {
          buttonPushCounter = 1;
        }
        delay(60);
      }
      if (currentMillis - previousMillis >= 250) 
      {
        previousMillis = currentMillis;
        if (ledState == LOW) 
        {
          ledState = HIGH;
        } 
        else 
        {
          ledState = LOW;
        }
        digitalWrite(ledPin, ledState);
      }
    }
    delay(50);  
  }
}

void processAccelGyro()
{
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02)
  {
    if (fifoCount < packetSize)
      return;
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.resetFIFO();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    Serial.print(mpuPitch);
    Serial.print("\t");
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    Serial.print(mpuRoll);
    Serial.print("\t");
    mpuYaw  = ypr[YAW] * 180 / M_PI;
    Serial.print(mpuYaw);
    Serial.print("\n");

    mpu.resetFIFO();

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    mpu.resetFIFO();

    x1 = -mpuPitch + 90;
    x2 = mpuRoll + 90;
    x3 = -mpuYaw + 90;
    
    c1 = constrain(x1, 0, 180);
    c2 = constrain(x2, 0, 180);
    c3 = constrain(x3, 0, 180);
    
    ServoPitch.write(c1);
    ServoRoll.write(c2);
    ServoYaw.write(c3);

    mpu.resetFIFO();
  }
}
