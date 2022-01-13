# Persistence of Vision Harddrive Clock  
## Operation  
- Face of clock is generated with a spinning harddrive platter with a slot cut in it from perimeter to center of disk.  
- LED array is installed behind the harddrive platter.  
- A hall effect sensor (OH090U) triggers an external interrupt on every rotation.  
- The microcontroller calculates the speed of the disk, and the timing for flashing the lights for the hands on each rotation.  

## Components  
- Microcontroller is an ESP8266 (Wemos D1 mini)  
- Using a hall effect sensor (OH090U) to trigger external interrupt and derive disk speed.  
- Using a FET (IRL540N) to flash the LEDs.  
- Using a Electronic Speed Controller to spin the harddrive motor.  

## Inspiration  
I was inspired by these projects:  
[Arduino HDD POV Clock](https://create.arduino.cc/projecthub/mircemk/arduino-hdd-pov-clock-persistence-of-vision-90a55a)  
[Hard Drive Persistence of Vision (HDPOV)](https://www.instructables.com/Hard-Drive-Persistence-of-Vision-HDPOV/)  

![Finished Project](https://github.com/clumsyCoder00/Persistence-of-Vision-Harddrive-Clock/blob/main/IMG_1824.jpg?raw=true)


