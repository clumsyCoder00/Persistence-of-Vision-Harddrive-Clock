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
const byte LEDpin = 12;       // D6
const byte interruptPin = 4;  // D2
const byte servoPin = 2;      // D4

unsigned long hands[7][3];

void IRAM_ATTR detectsRev ();
void setHand(unsigned long, long unsigned int (*)[7][3]);
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

unsigned tenths = 0;      // tenths
unsigned lastSec = 0;
unsigned int sec = 0;     // seconds hand
unsigned int mins = 0;    // minutes hand
unsigned int hrs = 0;     // hours hand
unsigned long offset = 0;

// NTPClient
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

void setup() {
  hands[0][2] = 1023;
  hands[1][2] = 85;
  hands[2][2] = 24;
  hands[3][2] = 12;
  hands[4][2] = 12;
  hands[5][2] = 12;
  hands[6][2] = 12;

  // Serial.begin(115200);
  myMotor.attach(servoPin);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), detectsRev, FALLING);
	pinMode(LEDpin, OUTPUT);
  analogWriteFreq(35000); // 1 – 1000Khz

  // get the motor started, ESC needs full throttle, then zero throttle signal to arm
  delay(1250);
  myMotor.write(180);
  delay(650);
  myMotor.write(0);

  // NTP
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    // Serial.print ( "." );
  }
  // Serial.println ( "connected" );
  timeClient.begin();
}

void loop() {
  thisClockCount = ESP.getCycleCount();
  
  if(interruptCounter>0){
    interruptCounter--;
    revoCycles = clockCount - lastClockCount;
    lastClockCount = clockCount;
    offset = revoCycles * 0.28; // greater offset value increases counter-clockwise shift. Use to adjust for sensor location.
  }

  setHand(thisClockCount-clockCount, &hands);

  // 80 000 000 = once per second
  // 8 000 000 = ten times per second
  // 4 000 000 = twenty times per second
  if((thisClockCount - i) >= 4000000) {
      timeClient.update();
      i = thisClockCount;

      // seconds
      sec = 59 - timeClient.getSeconds();
      if(sec != lastSec){
        lastSec=sec;
        tenths=0;
      }
      else
        tenths = (tenths + 1) % 20; // range of 0-19

      hands[0][0] = wrap(revoCycles * sec / 60 - revoCycles * tenths / 1200 + offset, revoCycles / 360, revoCycles, false);
      hands[0][1] = wrap(revoCycles * sec / 60 - revoCycles * tenths / 1200 + offset, revoCycles / 360, revoCycles,  true);

      // minutes
      mins = 59 - timeClient.getMinutes();
      hands[1][0] = wrap(revoCycles * mins / 60  + revoCycles * sec / 3600 + offset, revoCycles / 120, revoCycles, false);
      hands[1][1] = wrap(revoCycles * mins / 60  + revoCycles * sec / 3600 + offset, revoCycles / 120, revoCycles, true);

      // hours
      hrs = 12 - (timeClient.getHours()%12);
      hands[2][0] = wrap(revoCycles * hrs / 12 + revoCycles * mins / 720 + offset, revoCycles / 24, revoCycles, false);
      hands[2][1] = wrap(revoCycles * hrs / 12 + revoCycles * mins / 720 + offset, revoCycles / 24, revoCycles, true);

      // quarter indicators
      for(int j=3; j<7; j++){
        hands[j][0] = wrap(revoCycles * j * 15 / 60 + offset, revoCycles / 360, revoCycles, false);
        hands[j][1] = wrap(revoCycles * j * 15 / 60 + offset, revoCycles / 360, revoCycles,  true);
      }

    speedInput(myMotor);
  }
}

/* hands array
0 - seconds hand
1 - minutes hand
2 - hours hand
3 - ind 0
4 - ind 1
5 - ind 2
6 - ind 3

0 - secStart
1 - secStop
2 - brightness
*/
void setHand(unsigned long timeAtCall, unsigned long(*hands)[7][3]){
  unsigned long brightness = 0;

  for(int i = 6; i>=0; i--){ // cycle through array of hands to be displayed on clock face. Start with lowest priority
    if((*hands)[i][0] < (*hands)[i][1]){ // on and off events occurr on the same revolution
      if(timeAtCall >= (*hands)[i][0] && timeAtCall < (*hands)[i][1]) // test if currently in range of on event
        brightness = (*hands)[i][2]; // set brightness to corresponding hand
    }
    else{ // on and off events occur on this and next revolution
      if(timeAtCall >= (*hands)[i][0] || timeAtCall < (*hands)[i][1]) // test if currently in gap between this and next revolution
        brightness = (*hands)[i][2];
    }
  }

  if(brightness == 0) // no hands are to be on now, turn off LED
    digitalWrite(LEDpin, LOW);
  else
    analogWrite(LEDpin, brightness); // set PWM to brightness value of highest priority hand
}

// function to adjust delay values to wrap around to next revolution
// derive delay values based on center value and width of hand
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

// set motor speed by analog input
void speedInput(Servo& myMotor) {
  int val = map(analogRead(A0),1023,0,58,180);
  if (val != lastVal){
    lastVal = val;
    myMotor.write(val);
  }
}

IRAM_ATTR void detectsRev() {
  interruptCounter++;
  clockCount=ESP.getCycleCount();
}
