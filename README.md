# Multi-agent navigation using firebird 5
This repository provides a plug and play module to use firebird 5 for multiagent navigation
Features
* Interface for Serial communications
* Autocalibration of bot 
* Free Navigation in map
* Dynamic Obstacle avoidance

## How to use
Please refer [Firebird 5 AVR C code](https://github.com/RugvedKatole/firebird-5-Avr-code) for Programing ATMEGA2560

Install the following libraries from requirements.txt file

`pip3 install -r requirements.txt`

more to be added

### Bot Interface
This script communicates with the AVR microcontroller through serial communication on USB RS232. It sends PWM data to microcontroller when published on PWM topic and reads the sensor data from mcu and publishes it on respective topics
Run this file before running any other scripts

`rosrun syscon_fb5 bot_interface.py`

#### Topics and services
1. PWM : In this topic Right and Left PWM inputs are published which then are sent to bot
2. encoder: Here Encoder readings are published
3. prox_ir: Here a boolean vector of size 4 is published indicating true if there is a obstacle in vicinity false otherwise
5. wl: Here a boolean vector of size 3 is published indicating true if the is white line present
6. sharp_ir: This publishes the sharp_IR sensor readings i.e Distance in mm.
7. vel_to_PWM; This is a service to convert velocity to PWM commands.

### Vicon Motion Capture
Vicon Provides the pose of the robot at a frequency of 1000hz. It publishes the pose on the Topic name `/Vicon/<name of object>/<name of object>`

To stream this data from vicon download and setup [Vicon Bridge](https://github.com/ethz-asl/vicon_bridge)

Change the IP address given in the */vicon_bridge/launch/vicon.launch 

Default setting would be `<arg name="datastream_hostport" default="192.168.94.81:801"/>` change it to host PC ip keeping the port(801) same

To view published data run `rostopic echo /Vicon/<name of object>/<name of object>`

### AutoCalibration
To calibrate the bot follow the given steps to generate .csv file for bot calibration.

Note: This Setup uses Vicon System for calibration

There are 2 ways to proceed:

1] Install Vicon Bridge on Raspberry connected to Firebird 5 

2] Export the ROS MASTER URI of Rasp Pi to your PC(different than Vicon Host PC) where you would be running Vicon Bridge

After Completing above steps 
Run following commands

`roslaunch vicon_bridge vicon.launch`

`rosrun syscon_fb5 bot_interface.py`              incase you didn't do it earlier

`rosrun syscon_fb5 autocalibration.py`

Now the Bot will execute Different PWMs starting from 100 to 255 with a gap of 5 Each PWM will run for 5 seconds. You can change these default setting in autocalibration.py file.
The Bot Position and orientation will be stored in pos_x_y_z.csv file where x is direction (forward or backward), y is PWM and z is run id. Each PWM is executed 3 time to reduce the error.

All the CSV files can be found in Calibration_files folder

Now the run the Post processing file to process all the csv files

`python3 ~/<path to workspace/src/syscon_fb5/scripts/post_process.py`

The post_process.py will generate a yaml file containing the weights that should multiplied with velocity in order to get appropriate PWMs for right and left wheels.






