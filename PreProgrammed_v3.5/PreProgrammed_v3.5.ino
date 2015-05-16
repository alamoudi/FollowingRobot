#include <Boards.h>
#include <Firmata.h>

#include <Wire.h>                                      // interrupt based I2C library
#include <Servo.h>                                     // library to drive up to 12 servos using timer1
#include <EEPROM.h>                                    // library to access EEPROM memory
#include "IOpins.h"                                    // defines which I/O pin is used for what function

// define constants here
#define startbyte 0x0F                                 // for serial communications each datapacket must start with this byte

String voice;                                          // a string variable to store the voice activated mode
boolean sflag =false;

int iter = 0;                                          // define global variables here                                  
char mode='N';                                         // mode=0: I2C / mode=1: Radio Control / mode=2: Bluetooth / mode=3: Shutdown
int  lowbat=550;                                       // default low battery voltage is 5.5V
byte errorflag;                                        // non zero if bad data packet received
byte pwmfreq;                                          // value from 1-7
byte i2cfreq;                                          // I2C clock frequency can be 100kHz(default) or 400kHz
byte I2Caddress;                                       // I2C slave address
int lmspeed,rmspeed;                                   // left and right motor speeds -255 to +255
byte lmbrake,rmbrake;                                  // left and right brakes - non zero values enable brake
int lmcur,rmcur;                                       // left and right motor current
int lmenc,rmenc;                                       // left and right encoder values
int volts;                                             // battery voltage*10 (accurate to 1 decimal place)
int xaxis,yaxis,zaxis;                                 // X, Y, Z accelerometer readings
int deltx,delty,deltz;                                 // X, Y, Z impact readings 
int magnitude;                                         // impact magnitude
byte devibrate=50;                                     // number of 2mS intervals to wait after an impact has occured before a new impact can be recognized
int sensitivity=50;                                    // minimum magnitude required to register as an impact

byte RCdeadband=35;                                    // RCsignal can vary this much from 1500uS without controller responding
unsigned long time;                                    // timer used to monitor accelerometer and encoders

byte servopin[6]={7,8,12,13,5,6};                      // array stores IO pin for each servo
int servopos[6];                                       // array stores position data for up to 6 servos
Servo servo[6];                                        // create 6 servo objects as an array

long duration, cm;
//const int pingPin = 6;                                     //5 => Front, 6 => Right, 7 => Left 
long frontDist;
long rightDist;
long leftDist;
long distance[3];
int pingPin[] = {5,6,13};                              //using only one sensor for now
int brk;        //break; used to stop the motors
static byte alternate;

void setup()
{
 
  TCCR2B = TCCR2B & B11111000 | B00000110; pwmfreq=6;    // set timer 2 divisor to  256 for PWM frequency of    122.070312500 Hz
  pinMode(lmpwmpin,OUTPUT);                            // configure left  motor PWM       pin for output
  pinMode(lmdirpin,OUTPUT);                            // configure left  motor direction pin for output
  pinMode(lmbrkpin,OUTPUT);                            // configure left  motor brake     pin for output
  
  pinMode(rmpwmpin,OUTPUT);                            // configure right motor PWM       pin for output
  pinMode(rmdirpin,OUTPUT);                            // configure right motor direction pin for output
  pinMode(rmbrkpin,OUTPUT);                            // configure right motor brake     pin for output
  
  digitalWrite(RCspeedpin,1);                          // enable weak pullup resistor on input to prevent false triggering                   
  digitalWrite(RCsteerpin,1);                          // enable weak pullup resistor on input to prevent false triggering
  delay(100);
  
  int t1=int(pulseIn(RCspeedpin,HIGH,30000));          // read throttle/left stick
  int t2=int(pulseIn(RCsteerpin,HIGH,30000));          // read steering/right stick
  //servo[1].attach(servopin[1]);                                     // attach servo 
  MotorBeep(2);                                        // generate 2 beeps from the motors to indicate that preprogram already installed
 
  Serial.begin(9600);                                  // initialize serial communication for Ping Sensor
}

void loop()
{  
      
  /*int k;

  for (k=10; k>0; k--){
   SlightRight();
   Motors();
 }
 for (k=10; k>0; k--){
    SlightLeft();
    Motors();
 }*/ 
 //-------------------------Voice Activated------------------------------//
// while (Serial.available()){  //Check if there is an available byte to read
//  delay(10); //Delay added to make thing stable
//  char c = Serial.read(); //Conduct a serial read
//  if (c == '#') {break;} //Exit the loop when the # is detected after the word
//  voice += c; //Shorthand for voice = voice + c
//  } 
//  if (voice.length() > 0) {
//    Serial.println(voice);

//if(voice=='*go backward')
//  {
//    reverseMode();
//    return;
//  }
//  
//  if(voice=='*go forward')
//  {
//    forwardMode();
//    return;
//  }
//  
//  
//  if(voice == '*turn right'){
//    turnRight();
//    delay(100);
//    Shutdown();
//    return;
//  }
//  
//  if(voice == '*turn left'){
//    turnLeft();
//    delay(100);
//    Shutdown();
//    return;
//  }
//  
//  if(voice == '*Stop'){
//    stopMode();
//    Motors();
//    return;
//  }
//  
//  if(voice =='v')
//  {
//    SlightLeft();
//    Motors();
//    delay(100);
//   Shutdown();
// }
//  
//  if (voice=='q')                                         // if battery voltage too low
//  {
//    Shutdown();                                        // Shutdown motors and servos
//  }
// voice="";
//}
//-----------------------------------------------------------------------//

      pingTest();
  if (Serial.available() > 0)
  {  
    
    mode = Serial.read();
    //Serial.println(mode);
    
//    if (mode == 'e')
//    {
//      if (sflag == false)
//      sflag == true;
//      else 
//      sflag == false;
//    }
  if(mode=='r')
  {
    reverseMode();
    delay(1000);
    Shutdown();
    return;
  }
  //---------------------
  if(mode=='f')
  {
    forwardMode();
  //  delay(1000);
  //  Shutdown();
    return;
  }
  //---------------------
  
  if(mode == 'd'){
    turnRight();
   // delay(100);
    //Shutdown();
    return;
  }
  //---------------------
  if(mode == 'a'){
    turnLeft();
    //delay(100);
    //Shutdown();
    return;
  }
 if(mode =='m') {
   
   SlightRight();
   return;
  }
   if(mode =='n') {
   
   SlightLeft();
   return;
  }
  //---------------------
  if(mode == 's'){
    stopMode();
    Motors();
    return;
  }
  //---------------------
  if(mode=='v')
  {
    SlightLeft();
    Motors();
    delay(100);
   Shutdown();
 }
  //---------------------
  if (mode=='q')                                         // if battery voltage too low
  {
    Shutdown();                                        // Shutdown motors and servos
  }
  //---------------------
  }
//    for(servopos[1] = 0; servopos[1] < 180; servopos[1] += 1)  // goes from 0 degrees to 180 degrees 
//  {                                  // in steps of 1 degree 
//    servo[1].write(servopos[1]);              // servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  } 
//  for(servopos[1] = 180; servopos[1]>=1; servopos[1]-=1)     // goes from 180 degrees to 0 degrees 
//  {                                
//    servo[1].write(servopos[1]);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  }
}
//--------------------------------------------------- Ping Test ----------------------------------------------//

void pingTest(){
  int i=0;
  Serial.print("#");
  for(i=0; i<1; i++){                    //i = number of sensor 
    pinMode(pingPin[i], OUTPUT);
    digitalWrite(pingPin[i], LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin[i], HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin[i], LOW);
    pinMode(pingPin[i], INPUT);
    duration = pulseIn(pingPin[i], HIGH);
    distance[i] = duration / 29 / 2;
    
    Serial.print(distance[i]);
    Serial.print("~");
    //Serial.print("+");
    //delayMicroseconds(500);
    delay(15);
    //delay(100);
    //Serial.println();
  }  
      //Serial.print("~");  


if(distance[0] > 128 ){
    //forwardMode();
    //Serial.println("Forward");
  }
else {
  //Serial.println("Shut down");
   // Shutdown();
}
  delay(50);
  
  return;
  
}

//------------------------------------------------------ Modes ----------------------------------------------//

void forwardMode(){
  if(brk ==1){
    brk = 0;            //engage brake for quick slow down
    lmbrake = brk;
    rmbrake = brk;
    }
    int Speed= 1600;                                             // read throttle/left stick
    int Steer= 1500;                                              // read steering/right stick
  
    if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
    if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre
  
    if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
    if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
    
    Steer= Steer-1500;

    lmspeed = 57;
    rmspeed = 57;
        Motors();
}
//---------------------------------------------------//
void reverseMode(){
    if(brk ==1){
    brk = 0;            //engage brake for quick slow down
    lmbrake = brk;
    rmbrake = brk;
    }
    
    int Speed= 1400;                                             // read throttle/left stick
    int Steer= 1500;              // read steering/right stick

    if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
    if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre

    if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
    if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
  
    Steer= Steer-1500;
    lmspeed = -64;
    rmspeed = -64;
    Motors();
}
//---------------------------------------------------//
void stopMode(){
     brk = 1;            //engage brake for quick slow down
     lmbrake = brk;
     rmbrake = brk;
     lmspeed = 0;
     rmspeed = 0 ;
     Motors();
}
//---------------------------------------------------//
void turnRight(){
  if(brk ==1){
    brk = 0;            //engage brake for quick slow down
    lmbrake = brk;
    rmbrake = brk;
    }
    
    int Speed= 1400;                                            // read throttle/left stick
    int Steer= 1700;                                            // read steering/right stick

    if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
    if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre

    if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
    if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
  
    Steer=Steer-1500;

    lmspeed = -90;
    rmspeed = 90;
        Motors();
}
//---------------------------------------------------//
void turnLeft(){
  if(brk ==1){
    brk = 0;            //engage brake for quick slow down
    lmbrake = brk;
    rmbrake = brk;
    }
    
    int Speed= 1400;                                            // read throttle/left stick
    int Steer= 1300;                                            // read steering/right stick

    if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
    if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre

    if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
    if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
  
    Steer=Steer-1500;

      lmspeed = 90;
      rmspeed = -90;
          Motors();
}
//---------------------------------------------------//
void SlightLeft(){
     if(brk ==1){
    brk = 0;            //engage brake for quick slow down
    lmbrake = brk;
    rmbrake = brk;
    }
    int Speed= 1600;                                             // read throttle/left stick
    int Steer= 1500;                                              // read steering/right stick
  
    if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
    if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre
  
    if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
    if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
    
    Steer= Steer-1500;

    lmspeed = 57;
    rmspeed = -57;
        Motors();
}
//---------------------------------------------------//
void SlightRight(){
        if(brk ==1){
    brk = 0;            //engage brake for quick slow down
    lmbrake = brk;
    rmbrake = brk;
    }
    int Speed= 1600;                                             // read throttle/left stick
    int Steer= 1500;                                              // read steering/right stick
  
    if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
    if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre
  
    if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
    if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
    
    Steer= Steer-1500;

    lmspeed = -57;
    rmspeed = 57;
        Motors();
}
