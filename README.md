# conveyor_ws_sabhari_backup
Robotic waste sorting conveyor codebase as left by Sabhari Natarajan (called "conveyor_ws_sabhari_backup" in Sagamore) on August 31, 2021.

roslaunch realsense2_camera rs_camera.launch
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# Update by 2021-2022 MQP Team:
To run the new GUI:
Plug in both robot USBs and camera USB

Run:
realsense-viewer

Change camera exposure to around 170-200
Close realsense-viewer

Run:
roslaunch arm GUI_controller.launch 
roscd camera/scripts/
python3 camera_gui_node.py 
roscd arm/scripts
python3 master_controller.py 

Turn on the conveyor belt and set it to a speed of 15 in reverse.

cd conveyor_ws_sabhari_backup;cd src
roslaunch arm GUI_controller.launch;roscd camera/scripts/;python3 camera_gui_node.py;roscd arm/scripts;python3 master_controller.py