# Project for 2020 Fall SUSTech ME331 - Robot Modelling and Control

## The Aim of the Project
The project aims to build a 5-dof decoupled serial robot arm with spherical wrist(no rotation freedom)  
The robot arm is supposed to write and draw on plane or sphere surface, with a given input image.

## General Approach of the Project
### The hardware of the project
The hardware of the project includes the robot arm, using the Dynamixel AX-12A servo <https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/> and other structural connectors in Dynamixel Bioloid Premium <https://emanual.robotis.com/docs/en/edu/bioloid/premium/> kit and some 3D-print parts, with Dynamixel u2d2 <https://emanual.robotis.com/docs/en/parts/interface/u2d2/> for communication with a PC, and a PC running python scripts in Windows 10. The pen is a mark pen and using a spring to provide a constant force with the contact surface.

### The software of the project
The software part of the project is the python script. It uses opencv-python and numpy for image processing, then map the processed critical tarjectory point to the surface to draw on. Then inverse kinematics is then preformed to map the pose of the end-of-effector to configure space of the joints. Finally the Dynamixel SDK is called to drive the servos.  
The servo driver is a modified version of <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/#python-read-write-protocol-10>  
opencv-python: <https://pypi.org/project/opencv-python/>  version 4  
numpy: <https://numpy.org/>

## The actors of the project
(not list in order)  
CHEN Zhenyang  
ZHAO Yuntian  
GAO Chengyuan  
HE Rui  
CHE Haichuan  
LIN Zijun  

## Acknowledgement
We would like to thank Prof. FU Chenglong for his dedicated teaching and instrction and thank Prof. JIA Zhengzhong, LIN Shiyuan and LI Yifei in ROMA Lab for kindly advice and support of this project.
