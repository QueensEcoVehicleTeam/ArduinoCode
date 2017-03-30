//=================================================================================================
//Libraries
#include <Wire.h>
#include <StopWatch.h>
#include <Math.h>
#include <SPI.h>
#include <TFT.h>

//=================================================================================================
//Global constants
#define pin_tachometer  3  //tachometer hall effect sensor signal pin
#define pin_b_startstop 10  //button for starting and stopping the timer
#define pin_l_startstop 5
#define pin_b_lap       9  //button for incrementing lap counter
#define pin_l_lap       7  //led for lap count
#define pin_b_reset     8  //button for resetting timer and lap counter
#define pin_l_reset     9

#define tach_timeStep   1     //ms delay between tachometer readings
#define tach_speedCoeff 3600  //multiplier for km/h conversion
#define tach_repCount   3     //number of samples the tachometer takes

#define TFT_cs 1
#define TFT_dc 2
#define TFT_rst 3

 //=================================================================================================
 //QSM system class
class QSM_sys
{
  public:
  bool RIP;           //shows whether a race is
  float speed_kmph;   //car speed in kilometers per hour
  float incline_deg;  //car incline in degrees
  int lapCount;        //current lap

  StopWatch SW;       //instantiates the stopwatch timer class

  //Main functions
  QSM_sys();          //constructor for QSM system class (setup)
  void startRace();   //function the initializes the system for the race and begins the timer
  void raceLoop();    //continuously run during the race
  
  //Output functions
  void sendRFdata();  //sends the data to recieving arduino over RF
  void updateLCD();   //updates LCD screen with current values
  
  //Input functions
  float getSpeed();       //gets speed of car from tachometer
  float getIncline();     //gets car incline from accelerometer

  //accelerometer members and functions
  float gForceX, gForceY, gForceZ;
  double tiltY;
  void setupMPU();

  //stopwatch members and functions
  int  prevLapEndTime;
  int  lapTimes_arr[20];
  void SWbuttonFunctions(); //carries out fuctions performed by stop watch control buttons
  int  SWgetButtons();      //gets information from control buttons
  
};

QSM_sys::QSM_sys()
{ 
  Wire.begin();
  SW.reset();
  
  //Pin assignments
  pinMode(pin_tachometer, INPUT);
  pinMode(pin_b_startstop, INPUT);
  pinMode(pin_l_startstop, OUTPUT);
  pinMode(pin_b_reset, INPUT);
  pinMode(pin_l_reset, OUTPUT);
  pinMode(pin_b_lap, INPUT);
  pinMode(pin_l_lap, OUTPUT);

  //initialize variables
  lapCount = 0;
  speed_kmph = 0;
  incline_deg = 0;
  
}

void QSM_sys::startRace()
{
  
  speed_kmph = getSpeed();
  incline_deg = getIncline();
  
  SW.start();
  lapCount = 1;
  RIP = true;

}
//===========================================================================================================
//===========================================================================================================
//TACHOMETER PROCEDURE
float QSM_sys::getSpeed()
{

  int msCounter = 0; //measures wheel revolution time in ms
  
  for( int i = 0; i < tach_repCount; i++) //repeats the tachometer operation as many times as tach_repCount
  {
    //Serial.println("First Loop");
    //runs until the leading edge of the magnet passes the hall effect sensor
    while(digitalRead(pin_tachometer)==1)
    {
      delay(tach_timeStep);
    }
    //Serial.println("Second Loop"); //debug
    //triggered when the leading edge of the magnet passes the hall effect sensor
    while(digitalRead(pin_tachometer)==0)
    {
      msCounter++;
      //Serial.println(msCounter); //debug
      delay(tach_timeStep);
    }
    //Serial.println("Third Loop"); //debug
    //triggered when the magnet has fully passed the sensor
    //ends when the sensor sees the magnet again
    while(digitalRead(pin_tachometer)==1)
    {
      msCounter++;
      //Serial.println(msCounter); //debug
      delay(tach_timeStep);
    }
  }
  //Serial.println("Complete"); //debug
  msCounter = msCounter/tach_repCount; //calculates the average time per revolution
  
  return((1.6/(float)msCounter)*tach_speedCoeff); //returns the final value for speed in km/h

}

//===========================================================================================================
//=========================================================================================================== 
//STOPWATCH CONTROL BUTTON FUNCTIONS
void buttonFunction()
{
  if(digitalRead(pin_b_startstop))
    b_startstop_func();
  else if(digitalRead(pin_b_lap))
    b_lap_func();
  else if(digitalRead(pin_b_reset))
    b_reset_func();
}

void b_startstop_func()
{
  if(SW.state() == StopWatch::RESET)
  {
    RIP=true;
    SW.start();
    lapCount = 1;
  }
  else if (SW.state() == StopWatch::STOPPED)
  {
    SW.start();
    
  }
  //-------------------------
   else  //if the timer is currently running, stop it
  {
    SW.stop();
  }
}

void b_lap_func()
{
  int cur_t = SW.value();                           //gets the time at the exact moment the button is pressed
  lapTimes_arr[lapCount-1] = cur_t-prevLapEndTime;  //adds the lap time to the lap times array by subtracting the endtime of the last lap from the current time.
                                                    //lapCount-1 is because it starts on lap 1                                              
  prevLapEndTime = cur_t;                       //sets up the function for the next itteration
  lapCount++;                                   //itterates the lap counter
}

void b_reset_func()
{
  SW.reset();
  lap = 0
  RIP = false;
}
  


  
//===========================================================================================================
void QSM_sys::updateLCD()
{
  
}

void QSM_sys::setupMPU()
{
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

float QSM_sys::getIncline()
{
  long accelX, accelY, accelZ;
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  gForceX = (accelX) / 16384.0;
  gForceY = (accelY) / 16384.0; 
  gForceZ = (accelZ + 1302) / 16384.0;
  
  if (gForceY > 1)
  {
    gForceY = 1.00;
  }

  tiltX = 360 * (asin(gForceX))/(2*PI);
  tiltY = 360 * (asin(gForceY))/(2*PI);
  tiltZ = 360 * (asin9gForceZ))/(2*PI);
  
}

//=================================================================================================

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  QSM_sys sys;
  while(1)
  {
    sys.getButtons();
    Serial.println(sys.SW.value());
    delay(50);
  }
  
}

