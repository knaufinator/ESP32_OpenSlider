#include <TMCStepper.h>
#include <HardwareSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

using namespace TMC2209_n;
using namespace std;


BLEServer* pServer = NULL;
 
BLECharacteristic* pPauseCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
 
#define PAUSECHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

 
#define max_speed 4000 // steps per second
#define acceleration 100 // steps per second^2
#define num_steps 35000
#define num_stepsRz 1000
#define num_stepsRy 2000


#define DIAG_PIN        19         
#define EN_PIN          5         
#define DIR_PIN        23         
#define STEP_PIN         14          
#define SERIAL_PORT      Serial2    
#define DRIVER_ADDRESS   0b00       
#define R_SENSE          0.11f       
#define STALL_VALUE     50          

#define EN_PIN_2          27          
#define DIR_PIN_2        12          
#define STEP_PIN_2         25         
#define SERIAL_PORT_2      Serial    
#define DRIVER_ADDRESS_2   0b00       
#define R_SENSE_2          0.11f       
#define STALL_VALUE_2     50          

#define EN_PIN_3          13          
#define DIR_PIN_3        33          
#define STEP_PIN_3         32            
#define DRIVER_ADDRESS_3   0b00       
#define R_SENSE_3          0.11f      
#define STALL_VALUE_3     50          

MultiStepper StepperControl; 

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper1(1, STEP_PIN, DIR_PIN); // 1 = driver type, 2 = step pin, 3 = direction pin

TMC2209Stepper driver2(&SERIAL_PORT_2, R_SENSE_2, DRIVER_ADDRESS_2);
AccelStepper stepper2(1, STEP_PIN_2, DIR_PIN_2); // 1 = driver type, 2 = step pin, 3 = direction pin
 
HardwareSerial SerialPort_3(1);  
TMC2209Stepper driver3(&SerialPort_3, R_SENSE_3, DRIVER_ADDRESS_3);
AccelStepper stepper3(1, STEP_PIN_3, DIR_PIN_3); // 1 = driver type, 2 = step pin, 3 = direction pin
 
hw_timer_t * timer1 = NULL;
int32_t homeSpeed = 2000;
bool homed = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void clearError() {
  digitalWrite(EN_PIN, HIGH);
  delay(250);
  digitalWrite(EN_PIN, LOW);
  delay(250);
  digitalWrite(EN_PIN, HIGH);
  delay(250);
  digitalWrite(EN_PIN, LOW);

  digitalWrite(EN_PIN_2, HIGH);
  delay(250);
  digitalWrite(EN_PIN_2, LOW);
  delay(250);
  digitalWrite(EN_PIN_2, HIGH);
  delay(250);
  digitalWrite(EN_PIN_2, LOW);

  digitalWrite(EN_PIN_3, HIGH);
  delay(250);
  digitalWrite(EN_PIN_3, LOW);
  delay(250);
  digitalWrite(EN_PIN_3, HIGH);
  delay(250);
  digitalWrite(EN_PIN_3, LOW); 
}


//move x axis until rsense trigger 
void home()
{ 
  homed = false;
  driver.shaft(false);
  driver.VACTUAL(homeSpeed);

  delay(200);  
}

void Home()
{
  driver.shaft(false);
  driver.VACTUAL(homeSpeed);

  delay(100);  
  
  while(!digitalRead(DIAG_PIN) ) {  
    delay(20);  
  }
 
     driver.VACTUAL(0);
   
    clearError();
    homed = true;
    delay(20);  

    stepper1.setCurrentPosition(0);
    stepper1.moveTo(20);
    while(stepper1.distanceToGo() != 0)
    {
      stepper1.setSpeed(300);
      stepper1.runSpeed();
    }
    stepper1.setCurrentPosition(0);
}


class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
 
//to pause platform, send "1", to start send "0" string
class BlePauseCallback: public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *pCharacteristic) {
   
      string result = pCharacteristic->getValue().c_str();
      
      Serial.println("BlePauseCallback");
      Serial.println(pCharacteristic->getValue().c_str());
    
      int i = atoi(pCharacteristic->getValue().c_str());
    
      if(i == 0)
      {  
        //pauseEStop();
      }
      else if(i == 1)
      {
       // resumeEStop();
      }
    }    
};


void setupBle(){
  
  Serial.println("Starting BLE init!");

  BLEDevice::init("Open 6DOF Services");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pPauseCharacteristic = pService->createCharacteristic(
                                         PAUSECHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

 

           
  pPauseCharacteristic->addDescriptor(new BLE2902());

   
  pPauseCharacteristic->setCallbacks(new BlePauseCallback());
 
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising(); 
}



void setup() {

  //setup uarts for all 3 axis
  SerialPort_3.begin(115200, SERIAL_8N1, 2, 4); 
  SERIAL_PORT.begin(115200);
  SERIAL_PORT_2.begin(115200);
 
  setupBle();

  //x axis outputs
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT_PULLUP);
  
  //rz axis outputs
  pinMode(EN_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  //ry axis outputs
  pinMode(EN_PIN_3, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
   
  //setup x stepper
  stepper1.setMaxSpeed(max_speed);
  stepper1.setAcceleration(acceleration);
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(500); // mA
  driver.microsteps(8);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

   //setup rz stepper
  stepper2.setMaxSpeed(max_speed);
  stepper2.setAcceleration(acceleration);
  driver2.begin();
  driver2.toff(4);
  driver2.blank_time(24);
  driver2.rms_current(500); // mA
  driver2.microsteps(8);
  driver2.TCOOLTHRS(0xFFFFF); // 20bit max
  driver2.semin(5);
  driver2.semax(2);
  driver2.sedn(0b01);
  driver2.SGTHRS(STALL_VALUE);

  //setup ry stepper
  stepper3.setMaxSpeed(max_speed);
  stepper3.setAcceleration(acceleration);
  driver3.begin();
  driver3.toff(4);
  driver3.blank_time(24);
  driver3.rms_current(500); // mA
  driver3.microsteps(8);
  driver3.TCOOLTHRS(0xFFFFF); // 20bit max
  driver3.semin(5);
  driver3.semax(2);
  driver3.sedn(0b01);
  driver3.SGTHRS(STALL_VALUE);
   
  StepperControl.addStepper(stepper1);
  StepperControl.addStepper(stepper2);
  StepperControl.addStepper(stepper3);

  //clear any errors that may be in the stepper drivers
  clearError();

  //beging homing sequence
  Home();
}

bool moving = false;
bool pathFinished = false;
   
bool rZActive = false;
int rZdir = 1;

bool rYActive = false;
int rYdir = 1;


long gotoposition[3]; 

volatile long XInPoint=0;
volatile long YInPoint=0;
volatile long XOutPoint=0;
volatile long YOutPoint=0;  
volatile long totaldistance=0;
int flag=0; 
int temp=0;
int i,j;
unsigned long switch0=0;
unsigned long rotary0=0;
float setspeed=200;
float motorspeed;
float timeinsec;
float timeinmins;
volatile boolean TurnDetected;  
volatile boolean rotationdirection;  


void loop() {
  
/*     gotoposition[0]=num_steps;
    gotoposition[1]=2000;
    gotoposition[2]=0;
  
    int seconds = 60 * 60 * 2;


    float speed = num_steps / seconds;

    stepper1.setMaxSpeed(speed);
    StepperControl.moveTo(gotoposition);
    StepperControl.runSpeedToPosition(); */
   
 




}

