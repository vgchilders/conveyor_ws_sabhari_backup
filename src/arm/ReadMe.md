# Robotic Waste Sorting - Arm Package

* ```tool_control_table.py``` has the address defined for XC530W150 dynamixel used.

* ```tool_Dynamixel.py``` provides us *DynamixelMotor* class for high-level control dynamixels.

* ```StepperController``` folder has the codes which are written in the Teensy and provides high-level control of stepper motors

    * ```pins.h``` PIN numbers to which various wires for stepper and limit switches have been connected

    * ```StepperController.ino``` High-level control of stepper motors. (Serial input format : ```<x,y>``` )

* ```arm_controller.py``` integrates the dynamixel and stepper control and creates the services by which they can be controlled.

## Services

* ```arm/z_axis_set```: Controls the 2 dynamixels for z-axis motion (dynamixel_srv: float64)
* ```arm/gripper_set```: Controls the gripper dynamixel (dynamixel_srv: float64)
* ```arm/xy_axis_set```: Controlls the x and y axis stepper motors (stepper_srv: float64,float64)

## Notes
* ```setup.py``` and ```_init_.py``` are required for catkin_make so that the packages outside this folder can access the python scripts
* The stepper motor needs to be homed before it can start receiving inputs (Send ```<-99,-99>``` via arduino serial monitor or click start on GUI interface)