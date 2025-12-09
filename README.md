# ME405-Romi-Project

## Overview

This repository contains the software and documentation for a MicroPython Romi robot designed for navigation by usng coorperative multitasking, motor control, and sensors to follow a course. The Romi robot was built using a Nucleo L476RG. The line sensor, and bump sensors were purchased through Pololu. 

Please use the table of contents to navigate. 

## Map Course Description 

### Map 
The image shown below is the course the Romi was tasked to navigate through. To complete the course, the use of line following, state estimation and bump sensing was required. 

![Map Course](media/Game_Track-1.png)


## System_Architecture 

The Romi runs several tasks running using cotask.py to perform line following, state estimation, velocity PID, and map navigation. The following tasks were used to complete the project. Click each link for descriptions of each task. 

  - [PID_task](#PID_task)
  - [Encoder_task](#Encoder_task)
  - [LineSensor_task](#LineSensor_task)
  - [Follow_task](#Follow_task)
  - [task_imu](#task_imu)
  - [observer_task](#observer_task)
  - [map_task](#map_task)

Shared variables are created by using task_share.py to allow for communication through tasks without blocking. Each task is set up as a generator function with each generator having a corresponding priority and run speed based on the importance of the task. This is shown at the bottom the very bottom of the main.py file. 

### Task Diagram 

To show the concurrent tasks running for the Romi project Map course a task diagram was created. 



### Finite State Machine

Corresponding to each task, a finite state machine was constructed.









## Task Descriptions

Shown below are descriptions of each task used to complete the term project. 
Use this linnk to return back to [System_Architecture](#System_Architecture) 

### PID_task

### Encoder_task

### LineSensor_task

### Follow_task

### task_imu

### observer_task

### map_task



## Results




## Troubleshooting and calculations



## Takeaways 









