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
int LEDpin = 4;
const byte interruptPin = 13; //D7

void ICACHE_RAM_ATTR handleInterrupt ();
void setHand(unsigned long, unsigned long, unsigned long, boolean*, int, boolean, int, boolean, int);
unsigned long movingAvg(unsigned long*, unsigned long*, uint8_t, uint16_t, unsigned long);
unsigned long wrap(unsigned long, unsigned long, unsigned long, boolean);
void speedInput(Servo&);

Servo myMotor;
String incomingString;

volatile byte interruptCounter = 0;

int thisInterrupt = 0;
int lastInterrupt = 0;

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
const uint8_t revoLen = 6; // number of values used to average clock cycles per revolution
unsigned long revoArray[revoLen]; // declaring array
unsigned long revoSum = 0;
uint8_t revoIndex = 0;

unsigned long printLt = 0;

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
int minsBrt = 256;

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
  myMotor.attach(0);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);
	pinMode(LEDpin, OUTPUT);
  analogWriteFreq(50000); // 1 – 1000Khz

  // get the motor started, ESC needs full throttle, then zero throttle signal to arm
  delay(3000);
  myMotor.write(180);
  delay(3000);
  myMotor.write(0);
  delay(3000);
  myMotor.write(73);

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

  // thisInterrupt = digitalRead(interruptPin);
  // if (thisInterrupt != lastInterrupt)
  // {
  //   if (thisInterrupt == 0)
  //   {
  //     interruptCounter++;
  //     lastInterrupt = 0;
  //     clockCount = thisClockCount;
  //   }
  //   else
  //     lastInterrupt = 1;
  // }
  
  if(interruptCounter>0){
    interruptCounter--;
    revoTime = clockCount - lastClockCount;
    lastClockCount = clockCount;
    // testStart = ESP.getCycleCount();
    // revoAvg = movingAvg(revoArray, &revoSum, revoIndex, revoLen, revoTime);
    revoAvg = revoTime;
    revoIndex = (revoIndex + 1) % revoLen;
    offset = revoAvg/20;
    // testEnd = ESP.getCycleCount();
  }
  
  // if((thisClockCount - printLt) >= 26){
    // printLt = thisClockCount;
    setHand(thisClockCount-clockCount, secStart, secStop, &secON, secBrt, minsON, minsBrt, hrsON, hrsBrt);
    setHand(thisClockCount-clockCount, minsStart, minsStop, &minsON, minsBrt, secON, secBrt, hrsON, hrsBrt);
    setHand(thisClockCount-clockCount, hrsStart, hrsStop, &hrsON, hrsBrt, secON, secBrt, minsON, minsBrt);
  // }

  if((thisClockCount - i) >= 80000000) {
    if(timeClient.update()){
      i = thisClockCount;

      sec = timeClient.getSeconds();
      // sec = 45;
      secStart = wrap((revoAvg * sec)/60 + offset, revoAvg / 480, revoAvg, false);
      secStop  = wrap((revoAvg * sec)/60 + offset, revoAvg / 480, revoAvg,  true);

      mins = timeClient.getMinutes();
      // mins = 15;
      minsStart = wrap(((revoAvg * mins) / 60) + ((revoAvg * sec) / 3600) + offset, revoAvg / 120, revoAvg, false);
      minsStop  = wrap(((revoAvg * mins) / 60) + ((revoAvg * sec) / 3600) + offset, revoAvg / 120, revoAvg, true);

      hrs = (timeClient.getHours()%12);
      // hrs = 6;
      hrsStart = wrap(((revoAvg * hrs) / 12) + ((revoAvg * mins) / 720) + ((revoAvg * sec) / 43200) + offset, revoAvg / 24, revoAvg, false);
      hrsStop  = wrap(((revoAvg * hrs) / 12) + ((revoAvg * mins) / 720) + ((revoAvg * sec) / 43200) + offset, revoAvg / 24, revoAvg, true);
    }
    // Serial.println(testEnd - testStart);
  }

  // range: 62-145
  // 700 - 8900 rpm
  speedInput(myMotor);
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
  // interrupts();
}

void handleInterrupt() {
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
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch != 10){
      incomingString += ch;
    }
    else
    {
      Serial.println(incomingString);
      int val = incomingString.toInt();
      if (val > -1 && val < 181)
     {
       myMotor.write(val);
     }
     else
     {
       Serial.println("Value is NOT between 0 and 180");
       Serial.println("Error with the input");
     }
      incomingString = "";
    }
  }
}