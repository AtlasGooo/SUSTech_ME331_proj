# Project for 2020 Fall SUSTech ME331 - Robot Modelling and Control

## How to run the code
- make sure you have the same configure of the robot hardware, or at least similar. If your robot hardware setup is not identical as ours, please modify corresponding parameters in the code.
- make sure you have the following packages and have the SDK installed  
```numpy``` ```opencv-python```   
SDK installation<https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/python_windows/#python-windows> or <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/python_linux/#python-linux>
- Navigate to ```test/protocol1_0```
- run the command ```python proj.py```
- use command line to interact with the program
## The Aim of the Project
The project aims to build a 5-dof decoupled serial robot arm with spherical wrist(no rotation freedom)  
The robot arm is supposed to write and draw on plane or sphere surface, with a given input image.

## General Approach of the Project
### The hardware of the project
The hardware of the project includes the robot arm, using the Dynamixel AX-12A servo <https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/> and other structural connectors in Dynamixel Bioloid Premium <https://emanual.robotis.com/docs/en/edu/bioloid/premium/> kit and some 3D-print parts, with Dynamixel u2d2 <https://emanual.robotis.com/docs/en/parts/interface/u2d2/> for communication with a PC, and a PC running python scripts in Windows 10. The pen is a mark pen and using a spring to provide a constant force with the contact surface.

### The software of the project
The software part of the project is the python script. It uses opencv-python and numpy for image processing, then map the processed critical tarjectory point to the surface to draw on. Inverse kinematics is then preformed to map the pose of the end-of-effector to configure space of the joints. Finally the Dynamixel SDK is called to drive the servos.  
The servo driver is a modified version of <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/#python-read-write-protocol-10>  
opencv-python: <https://pypi.org/project/opencv-python/>  
numpy: <https://numpy.org/>  
**Most codes in this repo are servo SDK, and are clone from the Dynamixel repo.The code corresponding to what above is at <https://github.com/BigBillZhao/SUSTech_ME331_proj/blob/master/tests/protocol1_0/porj.py>**

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
