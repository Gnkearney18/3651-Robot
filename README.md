# Code in this project so far:

## 2 Sensors Test
Testing code when using 2 sensors together. 
- MPU92/60  Gyroscope
- VL53L4CX Time of Flight Sensor
Based off examples from corresponding libraries. 

## 2 Turn and Go
The final implementaion created for 2 iterations of map then turn, then return to initial posistion. This code was used in the demo for the project and can be seen in the videos.

## Button Test
Every wired button needs a test. I argue that the one button I have on the robot is by far the most important and I wanted to make sure it worked!

## Encoder Test
When implementing encoders, testing to make sure they all work is vitally important.

## VL53L4CD_Sat_HelloWorldTest
This is a demo library from the VL53L4CD arduino library. It is included as an example to ensure the distance sensor is working.

## Forward and Back 
The first implementation of putting the whole project togther. Maps a distance, turns a corresponding direction and maps another distance before returning to the original poisition from mapped memory.

## Motor Test
When implementing motors, testing to make sure they all work is vitally important.

## Funcitionizing
This code is a skeleton for the code developed for this robot. This class is what I used as an architecture to create the 2 Turn and Go and the Forward and Back. This class is going to be put into a library for cleaner code in the future

# Next Steps with this Project:
## Hardware
- After some hard use of the robot, there are some repairs to be done
- Create better motor mounts (The ones I designed worked for a time but are not the final implementation)
- Better battery accomadations
- And just a better battery


## Software
- Use encoder to implement straigtened driving rather than gyroscope
- Make supported library for this robot rather than a massive class with hundreds of lines
- Better named functions - I mean there is some (a lot) of sloppy names in here
- Dynamic mapping instead of hard coded

## Other
- Include writeup and 3D models in writeup and link to this project
