// arduino>>bluetooth
// D11   >>>  Rx
// D10   >>>  Tx
#include <SoftwareSerial.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float prevYpr[3];

int buttonPins[3] = {5, 6, 7};
int buttonStates[3];
int prevButtonStates[3];

SoftwareSerial serial(10, 11); // RX, TX

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void SetupComm();    //setups communication with airMouse program in pc
bool CheckConnection();
bool HasChanged();
void PrintButtonStates();
void GetButtonStates();
void SavePrevData();
void CalculateDiffs();
void WaitForWakeUp();
void Reset();
int loopNo = 0;
float xDeadzone, yDeadzone;
float diffs[3];
bool sleep = false;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
 //  Serial.begin(115200);
     serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    mpu.initialize();
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
       
        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
       // Serial.println(F("ok"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          serial.println(F("err1"));
          serial.print(devStatus);
          serial.println(F(")"));
        }
   // configure LED for output
    pinMode(LED_PIN, OUTPUT);

  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(49);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(-47);
    mpu.setXAccelOffset(-1588);
    mpu.setYAccelOffset(323);
    mpu.setZAccelOffset(972);
    
   //setup buttons
   for (int i = 0; i < 3; i++)
   {
     pinMode(buttonPins[i], INPUT);
     digitalWrite(buttonPins[i], HIGH);
   }
    
//  serial.println("alskdjflaskjflkkajs;lfjaslkfj");
    SetupComm();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
unsigned long long checkInterval = 10000;
unsigned long long timeSinceCheck = 0;
unsigned long long prevCheckTime = 0;
unsigned long long lastClickTime = 0;


void loop()
{
    
    //if program on pc does not respond
    timeSinceCheck = millis() - prevCheckTime;
    if (timeSinceCheck > checkInterval)
    {
      timeSinceCheck = 0;
      prevCheckTime = millis();
      if (!CheckConnection())
      {
        Reset();
        prevCheckTime = millis();
      }
    }
    
    GetButtonStates();
    //make it sleep if third buttong was clicked twice
     if (buttonStates[2] == 0 && prevButtonStates[2] == 1)
     {
        if (millis() - lastClickTime < 1000)
        {
          WaitForWakeUp();
          prevCheckTime = millis();
          lastClickTime = 0;
        }
        else 
          lastClickTime = millis();
     }
      
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
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
        
        
        CalculateDiffs();
        
        if (loopNo < 5 || HasChanged())
        {        
          serial.print(diffs[0]);
          serial.print(" ");
          serial.print(diffs[1]);
          serial.print(" ");
          serial.print(diffs[2]);
          serial.print(" ");
             
          //print button states
          PrintButtonStates();
          serial.println("");
          
          if (loopNo < 6)
            loopNo++;
            
          if (abs(diffs[0]) > xDeadzone)
            diffs[0] = 0;
          if (abs(diffs[2]) > yDeadzone)
            diffs[2] = 0;
        }
        
        SavePrevData();
        
         // blink LED to indicate activity
         blinkState = !blinkState;
         digitalWrite(LED_PIN, blinkState);
    }
}

void SetupComm()
{
    char ch;
    static bool was = false;
    serial.println("something");
    while (serial.available() && serial.read()); // empty buffer
    while (!serial.available() || serial.read() != 'w');                 // wait for data
    
    //get deadzones
    String str = "";
    char buff[20];
    serial.readBytesUntil('\n', buff, 20);
    str = buff;
   // serial.println(buff);
    /*xDeadzone = str.toFloat();
    serial.readBytesUntil('\n', buff, 20);
    //serial.println(buff);
    str = buff;
    yDeadzone = str.toFloat();*/
    yDeadzone = xDeadzone = 0.05;
    // send recognition code and wait for ready
    serial.println("air.0.1-ypr");
   // make sure it worked (returns 0 if so)
    //serial.println(str);
    serial.println("ok");
    
    while (!serial.available() || serial.read() != 'r'); // empty buffer again
}

bool CheckConnection()
{
  serial.print("c\r\n");
  char ch;
  unsigned long time = millis();
  unsigned long timePassed = 0;
  while (timePassed < 5000 && ch != 'c')
  {
    timePassed += millis() - time;
    time = millis();
    serial.readBytes(&ch, 1);
  }
  if (timePassed >= 5000)
    return false;
  else
    return true;
}

void GetButtonStates()
{
    for (int i = 0; i < 3; i++)
    {
        prevButtonStates[i] = buttonStates[i];
        buttonStates[i] = digitalRead(buttonPins[i]);
    }
}
  
 
void PrintButtonStates()
{
    for (int i = 0; i < 3; i++)
    {
      serial.print(buttonStates[i]);
      serial.print(" ");
    }
}

void SavePrevData()
{
    for (int i = 0; i < 3; i++)
        prevYpr[i] = ypr[i];
}

bool HasChanged()
{
    for (int i = 0; i < 3; i++)
    {
      if (prevButtonStates[i] != buttonStates[i])
        return true;
    }
       
    if (abs(diffs[0]) > xDeadzone)
      return true;
    if (abs(diffs[2]) > yDeadzone)
      return true;
    return false;
}

void CalculateDiffs()
{
    for (int i = 0; i < 3; i++)
        diffs[i] += (ypr[i] - prevYpr[i]) * 180/M_PI;
}

void Reset()
{
    mpu.setSleepEnabled(true);
    SetupComm();
    mpu.setSleepEnabled(false);
    mpu.resetFIFO();
    loopNo = 0;
}

void WaitForWakeUp()
{
   mpu.setSleepEnabled(true);
   lastClickTime = 0;
   bool wait = true;
   while (wait)
   {
      GetButtonStates();
      if (buttonStates[2] == 0 && prevButtonStates[2] == 1)
      {
        if (millis() - lastClickTime < 1000)
        {
          wait = false;
         // serial.println("woke up");
          lastClickTime = 0;
          mpu.setSleepEnabled(false);
          mpu.resetFIFO();
        }
        else
          lastClickTime = millis();
      }
      //if program on pc does not respond
      timeSinceCheck = millis() - prevCheckTime;
      if (timeSinceCheck > checkInterval)
      {
        timeSinceCheck = 0;
        prevCheckTime = millis();
        if (!CheckConnection())
        {
          Reset();
          prevCheckTime = millis();
          wait = false;    //begin from the start
        }
      }
   }
}
