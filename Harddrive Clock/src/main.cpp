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
const char *ssid     = "SSID";
const char *password = "PASSWORD";

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

volatile unsigned long clockCount;
volatile unsigned long lastClockCount;
unsigned long thisClockCount;
unsigned long revoCycles;
unsigned long i = 0;

// temp
unsigned long testStart;
unsigned long testEnd;
// temp

// tenths
unsigned tenths = 0;
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
int minsBrt = 85;

// hours hand
unsigned int hrs = 0;
unsigned long hrsStart = 0;
unsigned long hrsStop = 0;
boolean hrsON = false;
int hrsBrt = 24;

// indicators
unsigned int ind[4] = {0};
unsigned long indStart[4] = {0};
unsigned long indStop[4] = {0};
boolean indON[4] = {false};
int indBrt = 18;


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
  analogWriteFreq(45000); // 1 – 1000Khz

  // get the motor started, ESC needs full throttle, then zero throttle signal to arm
  delay(1250);
  myMotor.write(180);
  delay(650);
  myMotor.write(0);
  delay(650);
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
    revoCycles = clockCount - lastClockCount;
    lastClockCount = clockCount;
    // testStart = ESP.getCycleCount();
    // revoCycles = 4 000 000 when spinning as slow as possible
    offset = revoCycles * 0.28; // higher = counter-clockwise shift. example: .22 = ahead by 3 sec .25 = ahead by 1 sec
    // testEnd = ESP.getCycleCount();
  }

  setHand(thisClockCount-clockCount, secStart, secStop, &secON, secBrt, minsON, minsBrt, hrsON, hrsBrt);
  setHand(thisClockCount-clockCount, minsStart, minsStop, &minsON, minsBrt, secON, secBrt, hrsON, hrsBrt);
  setHand(thisClockCount-clockCount, hrsStart, hrsStop, &hrsON, hrsBrt, secON, secBrt, minsON, minsBrt);

  for(int j=0; j<4; j++){
    //      full revo time            time on   time off  is on brightness  others
    setHand(thisClockCount-clockCount, indStart[j], indStop[j], &indON[j], indBrt, minsON, minsBrt, hrsON, hrsBrt);
  }

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

      secStart = wrap((revoCycles * sec)/60 - ((revoCycles/1200) * tenths) + offset, revoCycles / 360, revoCycles, false);
      secStop  = wrap((revoCycles * sec)/60 - ((revoCycles/1200) * tenths) + offset, revoCycles / 360, revoCycles,  true);

      mins = 59 - timeClient.getMinutes();
      // mins = 0;
      minsStart = wrap(((revoCycles * mins) / 60) + ((revoCycles * sec) / 3600) + offset, revoCycles / 120, revoCycles, false);
      minsStop  = wrap(((revoCycles * mins) / 60) + ((revoCycles * sec) / 3600) + offset, revoCycles / 120, revoCycles, true);

      hrs = 12 - (timeClient.getHours()%12);
      // hrs = 6;
      hrsStart = wrap(((revoCycles * hrs) / 12) + ((revoCycles * mins) / 720) + ((revoCycles * sec) / 43200) + offset, revoCycles / 24, revoCycles, false);
      hrsStop  = wrap(((revoCycles * hrs) / 12) + ((revoCycles * mins) / 720) + ((revoCycles * sec) / 43200) + offset, revoCycles / 24, revoCycles, true);

      for(int j=0; j<4; j++){
        indStart[j] = wrap((revoCycles * j * 15/60) + offset, revoCycles / 360, revoCycles, false);
        indStop[j]  = wrap((revoCycles * j * 15/60) + offset, revoCycles / 360, revoCycles,  true);
      }
    }

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

void speedInput(Servo& myMotor) {
  int val = map(analogRead(A0),1024,7,54,180);
  if (val != lastVal){
    lastVal = val;
    myMotor.write(val);
  }
}
