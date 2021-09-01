# Robotic Waste Sorting

**arm** package : Code to control the dynamixel motors (python) and stepper motors (python via Tennsy). Motors can be controlled using rosservices.

**gui** package : PyQt based GUI. Connects the various GUI buttons to the ROS services and also displays the realsense camera feed.

Use ```roslaunch arm GUI_controller.launch``` to start the realsense node, dynamixel control services, stepper control services and GUI

## Axis Configurations

* X-axis origin : Closest to lab entrance
* Y-axis origin : Closest to franka emika

## Notes

* Current X-axis homing is at the **X-axis maximum** on not X-axis origin due the drag chain cable wire carrier.
* In GUI, the negative of x is used and hence if <1000,100> is sent, the Teensy receives <-1000,100>.

## To-Do

* Rewire the setup using sheilded cables.
* Use PCB instead of breadboard.
* Fix the Flexible Drag Chain Cable Wire Carrier so that the stepper can home at X-axis origin.
* Add codes in ```StepperController.ino``` for Y-axis control
* The distance moved by stepper and dynamixel need to be adjusted to move in cms and not motor-specific units.