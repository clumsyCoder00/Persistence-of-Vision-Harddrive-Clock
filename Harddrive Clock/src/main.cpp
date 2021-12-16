#include <Arduino.h>
#include <Servo.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/*
simple interrupts
https://techtutorialsx.com/2016/12/11/esp8266-external-interrupts/#comments
*/

// wireless network data
const char *ssid     = "watkins";
const char *password = "abc123ab";

const long utcOffsetInSeconds = - 14400; // -4 hrs with daylight harvesting, -18000 without

// pins for hardware interfacing
const byte LEDpin = 12; //D6
const byte interruptPin = 4; // D2
const byte servoPin = 2; // D4

void IRAM_ATTR detectsRev ();
void setHand(unsigned long, unsigned long, unsigned long, boolean*, int, boolean, int, boolean, int);
unsigned long movingAvg(unsigned long*, unsigned long*, uint8_t, uint16_t, unsigned long);
unsigned long wrap(unsigned long, unsigned long, unsigned long, boolean);
void speedInput(Servo&);

Servo myMotor;
String incomingString;
int lastVal = 0;

volatile byte interruptCounter = 0;

// int thisInterrupt = 0;
// int lastInterrupt = 0;

volatile unsigned long clockCount;
volatile unsigned long lastClockCount;
unsigned long thisClockCount;
unsigned long revoTime;
unsigned long i = 0;

// temp
unsigned long testStart;
unsigned long testEnd;
// temp


unsigned long revoAvg = 0; // average clock cycles per revolution, rolling average of revoLen values
// const uint8_t revoLen = 6; // number of values used to average clock cycles per revolution
// unsigned long revoArray[revoLen]; // declaring array
// unsigned long revoSum = 0;
// uint8_t revoIndex = 0;

// tenths
unsigned tenths = 0;
//int secTenths = 0;
unsigned lastSec = 0;

// seconds hand
unsigned int sec = 0;       // store time - seconds
unsigned long secStart = 0; // cycles from reference to turn on LED
unsigned long secStop = 0;  // cycles from reference to turn off LED
boolean secON = false;      // is the LED on?
int secBrt = 1023;          // pwm value for hand brightness

// minutes hand
unsigned int mins = 0;
unsigned long minsStart = 0;
unsigned long minsStop = 0;
boolean minsON = false;
int minsBrt = 105;

// hours hand
unsigned int hrs = 0;
unsigned long hrsStart = 0;
unsigned long hrsStop = 0;
boolean hrsON = false;
int hrsBrt = 24;

unsigned long offset = 0;

// NTPClient
// Define NTP Client to get time
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


void setup() {
  Serial.begin(115200);
  myMotor.attach(servoPin);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), detectsRev, FALLING);
	pinMode(LEDpin, OUTPUT);
  analogWriteFreq(25000); // 1 – 1000Khz

  // get the motor started, ESC needs full throttle, then zero throttle signal to arm
  delay(1500);
  myMotor.write(180);
  delay(750);
  myMotor.write(0);
  delay(750);
  myMotor.write(55);

// NTP
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
    Serial.println ( "connected" );
  timeClient.begin();
}

void loop() {
  thisClockCount = ESP.getCycleCount();
  
  if(interruptCounter>0){
    interruptCounter--;
    revoTime = clockCount - lastClockCount;
    lastClockCount = clockCount;
    // testStart = ESP.getCycleCount();
    // revoAvg = movingAvg(revoArray, &revoSum, revoIndex, revoLen, revoTime);
    // revoIndex = (revoIndex + 1) % revoLen;
    revoAvg = revoTime;
    // revoAvg = 4 000 000 when spinning as slow as possible
    offset = revoAvg * 0.28; // higher, farther counter-clockwise .22 ahead by 3 sec .25 ahead by 1 sec
    // testEnd = ESP.getCycleCount();
  }

  setHand(thisClockCount-clockCount, secStart, secStop, &secON, secBrt, minsON, minsBrt, hrsON, hrsBrt);
  setHand(thisClockCount-clockCount, minsStart, minsStop, &minsON, minsBrt, secON, secBrt, hrsON, hrsBrt);
  setHand(thisClockCount-clockCount, hrsStart, hrsStop, &hrsON, hrsBrt, secON, secBrt, minsON, minsBrt);

  // 80 000 000 = once per second
  // 8 000 000 = ten times per second
  // 4 000 000 = twenty times per second
  if((thisClockCount - i) >= 4000000) {
    if(timeClient.update()){
      i = thisClockCount;

      sec = 59 - timeClient.getSeconds();

      if(sec != lastSec){
        lastSec=sec;
        tenths=0;
      }
      else
        tenths = (tenths + 1) % 20;
      
      // sec = 15;
      //secTenths = sec + tenths; // increment in range from 0 - 9
      // Serial.println((revoAvg/600) * tenths);
      secStart = wrap((revoAvg * sec)/60 - ((revoAvg/1200) * tenths) + offset, revoAvg / 480, revoAvg, false);
      secStop  = wrap((revoAvg * sec)/60 - ((revoAvg/1200) * tenths) + offset, revoAvg / 480, revoAvg,  true);

      mins = 59 - timeClient.getMinutes();
      // mins = 0;
      minsStart = wrap(((revoAvg * mins) / 60) + ((revoAvg * sec) / 3600) + offset, revoAvg / 120, revoAvg, false);
      minsStop  = wrap(((revoAvg * mins) / 60) + ((revoAvg * sec) / 3600) + offset, revoAvg / 120, revoAvg, true);

      hrs = 12 - (timeClient.getHours()%12);
      // hrs = 6;
      hrsStart = wrap(((revoAvg * hrs) / 12) + ((revoAvg * mins) / 720) + ((revoAvg * sec) / 43200) + offset, revoAvg / 24, revoAvg, false);
      hrsStop  = wrap(((revoAvg * hrs) / 12) + ((revoAvg * mins) / 720) + ((revoAvg * sec) / 43200) + offset, revoAvg / 24, revoAvg, true);
    }

    // range: 62-145
    // 700 - 8900 rpm
    speedInput(myMotor);
    // Serial.print("test: ");
    // Serial.println(testEnd - testStart);
  }
}

unsigned long wrap(unsigned long center, unsigned long wing, unsigned long ceiling, boolean additive){
  unsigned long sum = 0;
  // offset puts the center value past the ceiling, wrap it around to the beginning
  if(center > ceiling)
    center = center - ceiling;

  // secStop
  if(additive)
    sum = center + wing;

  // secStart
  else{
    if(wing > center){
      sum = center + ceiling;
      sum = sum - wing;
    }
    else
      return center - wing;
  }

  if(sum >= ceiling)
    return sum - ceiling;
  else
    return sum;
}

void setHand(unsigned long timeAtCall, unsigned long startTime, unsigned long stopTime, boolean *isON, int bright, boolean other1ON, int other1Brt, boolean other2ON, int other2Brt){
  // noInterrupts();
  if(startTime < stopTime){
    if(timeAtCall >= startTime && timeAtCall < stopTime){
      if(!*isON){
        analogWrite(LEDpin, bright);
        *isON = true;
      }
    }
    else{
      if(*isON){
        *isON = false;
          if(!other1ON && !other2ON)
            digitalWrite(LEDpin, LOW);
          else if (other1ON)
            analogWrite(LEDpin, other1Brt);
          else
            analogWrite(LEDpin, other2Brt);
      }
    }
  }

  else{
    if(timeAtCall >= startTime || timeAtCall < stopTime){
      if(!*isON){
        analogWrite(LEDpin, bright);
        *isON = true;
      }
    }
    else{
      if(*isON){
        *isON = false;
          if(!other1ON && !other2ON)
            digitalWrite(LEDpin, LOW);
          else if (other1ON)
            analogWrite(LEDpin, other1Brt);
          else
            analogWrite(LEDpin, other2Brt);
      }
    }
  }
}

IRAM_ATTR void detectsRev() {
  interruptCounter++;
  clockCount=ESP.getCycleCount();
}

unsigned long movingAvg(unsigned long *ptrArrNumbers, unsigned long *ptrSum, uint8_t pos, uint16_t len, unsigned long nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}

void speedInput(Servo& myMotor) {
  // 7 - 1024
  // 60*1/(4600000/80000000) = RPM
  int val = map(analogRead(A0),1024,7,54,180);
  if (val != lastVal){
    lastVal = val;
    myMotor.write(val);
    // Serial.println(analogRead(A0));
  }
}