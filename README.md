# ME405-Romi-Project

## Overview

This repository contains the software and documentation for a MicroPython Romi robot designed for navigation by usng coorperative multitasking, motor control, and sensors to follow a course. The Romi robot was built using a Nucleo L476RG. The line sensor, and bump sensors were purchased through Pololu. 

**Please use the table of contents in the top right corner to navigate the page.** 

 _The collaborators on this project are **Eric Lee, Roman Ruettimann, and Jonathan Enrique Corvera.** All three students each contributed to the software and hardware of the code equally._ 

## Map Course Description 

### Map 
The image shown below is the course the Romi was tasked to navigate through. To complete the course, the use of line following, state estimation and bump sensing was required. Each large black dot indicates a checkpoint that needs to be reached to complete the assignment. Red cups are placed inside the dotted circles. When hit out of the dotted circle, 3 seconds is deducted from the team's official time. 

![Map Course](media/Game_Track-1.png)

### Romi 

The robot used in this project is shown below. This robot consists of a chassis 
![IMG_6576](https://github.com/user-attachments/assets/ed25e055-c15b-46d2-824f-2e3d41bdcd17)


Component List
| Quantity | Component |
| -------- | --------- |
| 4 |  M2.5 x 8mm Standoff | 
| 4	| M2.5 x 10mm Standoff |
| 4 | M2.5 x 30mm Standoff |
| 4 |	M2.5 x 6mm Socket Head Cap Screw |
| 4	| M2.5 x 8mm Socket Head Cap Screw |
| 4	| M2.5 x 10mm Socket Head Cap Screw |
| 8	| M2.5 Nylon Lock Nuts |
| 8	| M2.5 Nylon Washer | 
| 1	| Acrylic Romi-to-Shoe Adapter |
| 1	| BNO055 IMU Breakout Board | 
| 1	| Unmodified Shoe of Brian |
| 1	| Modified Shoe of Brian |
| 2	| Nucleo L476RG |
| 1	| Romi Chassis w/ Motors, Encoders, Wheels, and Casters |

## System_Architecture 

The Romi runs several tasks running using cotask.py to perform line following, state estimation, velocity PID, and map navigation. The following tasks were used to complete the project. Click each link for descriptions of each task. 

  - [PID_task](#PID_task)
  - [Encoder_task](#Encoder_task)
  - [LineSensor_task](#LineSensor_task)
  - [Follow_task](#Follow_task)
  - [task_imu](#task_imu)
  - [observer_task](#observer_task)
  - [test_observer_task](#test_observer_task)
  - [map_task](#map_task)

Shared variables are created by using task_share.py to allow for communication through tasks without blocking. Each task is set up as a generator function with each generator having a corresponding priority and run speed based on the importance of the task. This is shown at the bottom the very bottom of the main.py file. 

### Task Diagram 

To show the concurrent tasks running for the Romi project Map course a task diagram was created. 

https://github.com/user-attachments/assets/12b63d06-b12a-43de-abc6-85b22fbb766f

A video of the follow task is shown 

### Finite State Machine

Corresponding to each task, a finite state machine was constructed.









## Task Descriptions

Shown below are descriptions of each task used to complete the term project. 
Use this linnk to return back to [System_Architecture](#System_Architecture) 

### PID_task

This task runs a closed-loop wheel velocity when the state is set to 2 or at 4. This PID was calibrated with individual kp, ki, and kd. The code includes a target speed regulator and set PID. Furthermore the controller includes a feedforward option in order to counter any disturbances. This feature intends to reduce any overshoot. 

### Encoder_task
The Encoder task updates the positions and velocities of the encoders based on open loop logginng. The values are converted into radians per second and filtered in order to accurately record the positions and velocity. 

### LineSensor_task
This task is responsible for running a simple calibration for the line sensor. By calling functions from the LineSensorArray class, the Romi is able to run calibrations for both the white and black lines. This is useful to instruct the robot to follow a line. 

### Follow_task
The follow task runs after the LineSensor task by taking the calibrated data and using it to follow the line. The centroid is calculated and adjusted based on the 5 IR line sensor signals. The motors are adjusted accordingly in order to keep the line centered to the sensor. 

### task_imu
The IMU task is responsible for indicated the heding, and yaw rate found using the IMU sensor. The data is recorded and the used for the observer tasks. 

### observer_task
By running an rk4 solver, the encoder angular velocity, speed of the Romi, turning angle, and turning speed are all calculated. The observer task is used for state estimation to accurately control the robots distance and turning angles. The state estimation is especially useful in the course to navigate through obstacles with no lines. 

### test_observer_task
This task runs tests for the state estimation in order to confirm whether the romi is calibrated correctly to accurately move a certain distance or turn a certain angle. 

### map_task
The map task is responsible for navigating through the course shown in the first image above. The map uses line following, state estimation, and pid control to help complete each section of the course. Each task is called using states which are shared throughout all the tasks. The task is responsible for calibrating the line sensor, follow the line, and switch between line following and state esitmation. 


## Results

Line Sensor Following. The video below shows the Romi following a path 
<video src="path/media/IMG_6451.mp4" width="320" height="240" controls></video>


## Troubleshooting and calculations
Main issues pertained to the line following of the robot. At the beginning the line sensor would not follow the line accurately, usually overshooting or lacking the response time to quickly adjust back to the line.




## Takeaways 









