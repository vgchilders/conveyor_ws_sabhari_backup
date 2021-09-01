# Robotic Waste Sorting - GUI Package

* ```GUI_Template.ui``` is the PyQt file. It can be edited using the pyQt GUI.

* ```GUI_Template.py``` is auto generated. **Do not manually edit this file**. Use ```pyuic5 GUI_Template.ui -o GUI_Template.py```for convert from .ui to .py.

* ```GUI.py``` used the ```GUI_Template.py``` and creates connections between the buttons and the functions calling rosservices. It also display the video feed from realsense camera.


## Notes
* ```setup.py``` and ```_init_.py``` are required for catkin_make so that the packages outside this folder can access the python scripts
* The stepper motor needs to be homed before it can start receiving inputs (Send ```<-99,-99>``` via arduino serial monitor or click start on GUI interface)