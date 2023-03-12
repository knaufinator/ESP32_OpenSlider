#include <TMCStepper.h>
#include <HardwareSerial.h>
#include <AccelStepper.h>

using namespace TMC2209_n;
 
#define max_speed 2000 // steps per second
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

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(1, STEP_PIN, DIR_PIN); // 1 = driver type, 2 = step pin, 3 = direction pin

TMC2209Stepper driver2(&SERIAL_PORT_2, R_SENSE_2, DRIVER_ADDRESS_2);
AccelStepper stepper2(1, STEP_PIN_2, DIR_PIN_2); // 1 = driver type, 2 = step pin, 3 = direction pin
 
HardwareSerial SerialPort_3(1);  
TMC2209Stepper driver3(&SerialPort_3, R_SENSE_3, DRIVER_ADDRESS_3);
AccelStepper stepper3(1, STEP_PIN_3, DIR_PIN_3); // 1 = driver type, 2 = step pin, 3 = direction pin
 
hw_timer_t * timer1 = NULL;
int32_t homeSpeed = 2000;
bool homed = false;

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


void setup() {

  //setup uarts for all 3 axis
  SerialPort_3.begin(115200, SERIAL_8N1, 2, 4); 
  SERIAL_PORT.begin(115200);
  SERIAL_PORT_2.begin(115200);
 
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
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(acceleration);
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

  //clear any errors that may be in the stepper drivers
  clearError();

  //beging homing sequence
  home();
}

bool moving = false;
bool pathFinished = false;
   
bool rZActive = false;
int rZdir = 1;

bool rYActive = false;
int rYdir = 1;

void loop() {
  
  //stop homing sequence when read pin, convert this to interupt
  if (!homed && digitalRead(DIAG_PIN) ) {  
    driver.VACTUAL(0);
    driver2.VACTUAL(0);
    driver3.VACTUAL(0);
    clearError();
    homed = true;
    delay(20);  
  }

  //start test sequences for all 3 motors
  if(homed && !pathFinished && !moving)
  {
    stepper.moveTo(num_steps);
    moving = true;
  }
 
  if(homed && !rZActive)
  {
    stepper2.moveTo(rZdir * num_stepsRz);
    rZActive = true;
  }

  if(homed && !rYActive)
  {
    stepper3.moveTo(rYdir * num_stepsRy);
    rYActive = true;
  }

  //when test sequence is up on x, just stop it
  if(moving && stepper.distanceToGo() == 0 && !pathFinished)
  {
    moving = false;
    pathFinished = true;
  }
 
  //for ry, rz, have them reverse direction, next itteration, and repeat
  if(rYActive && stepper3.distanceToGo() == 0)
  {
    //turn around 
    rYdir = -rYdir;
    rYActive = false;
  }

  if(rZActive && stepper2.distanceToGo() == 0)
  {
    //turn around 
    rZdir = -rZdir;
    rZActive = false;
  }
 


  //run each motor if needed, only start when home sequence has completed
  if(homed && moving && !pathFinished)
  {
      stepper.run();  
  }   

  if(homed && rZActive )
  {
      stepper2.run();  
  }   

  if(homed && rZActive )
  {
      stepper3.run();
  }  
}

