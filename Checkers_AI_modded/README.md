# This is a modified vesrion of checkers AI
Original git:
```Bash
https://github.com/Hsankesara/Draughts-AI
```

Modifications:
Small modifications have been made to the checkers.py file using new
python classes to publish the current gameboard state to a ROS node. 
This gameboard state is an array containing the location and occupancy
colour of each checker on the board.



## To run detectCheckers:

#### Compile workspace:
```Bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

#### If not done yet:
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

