# This is a modified vesrion of checkers AI
Original git:
```Bash
https://github.com/Hsankesara/Draughts-AI
```

Modifications:
Small modifications have been made to the game to publish the current gameboard state to a ROS node.
An additional file is created to detect checkers using image segmentation and interacts with the main game
if enabled through user input. This gameboard state is an array containing the location and occupancy
colour of each checker on the board.



## To run detectCheckers:

#### Compile workspace:
```Bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

#### If camera is not calibrated:
<details>
  
```Bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
```
</details>

#### Launch webcam:
```Bash
roslaunch usb_cam usb_cam-test.launch
```

#### Launch detection program:
```Bash
python3 detectCheckers.py
```

#### Launch main program with detection:
```Bash
python3 main.py
```


# RUN ROSBAG FOR TESTING:
```Bash
cd ~/catkin_ws/src/robot_checkers/Checkers_AI_modded
rosbag play --pause my_bag.bag
```

