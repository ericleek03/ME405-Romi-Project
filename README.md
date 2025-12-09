# ME405-Romi-Project

## Overview

This repository contains the software and documentation for a MicroPython Romi robot designed for navigation by usng coorperative multitasking, motor control, and sensors to follow a course. The Romi robot was built using a Nucleo L476RG. The line sensor, and bump sensors were purchased through Pololu. 

Please use the table of contents to navigate. 

## System Architeture 

The Romi runs several tasks running using cotask.py to perform line following, state estimation, velocity PID, and map navigation. The following tasks were used to complete the project. Each link will explain what the code will do.

  - [PID_task](PID_task)
  - Encoder_task
  - LineSensor_task
  - Follow_task
  - task_imu
  - observer_task
  - map_task

Shared variables are created by using task_share.py to allow for communication through tasks without blocking. Each task is set up as a generator function with each generator having a corresponding priority and run speed based on the importance of the task. This is shown at the bottom the very bottom of the main.py file. 

### Task Diagram 

To show the concurrent tasks running for the Romi project Map course a task diagram was created. 



### Finite State Machine

Corresponding to each task, a finite state machine was constructed.





## Map Course Description 

### Map 
The image shown below is the course the Romi was tasked to navigate through. To complete the course, the use of line following, state estimation and bump sensing was required. 

![Map Course](media/Game_Track-1.png)







## PID_task

