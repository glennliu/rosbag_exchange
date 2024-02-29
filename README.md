# Rosbag converter

This is a rosbag package to transfer data in rosbag into sequence folders.

To install, create a ros workspace and run ```catkin_make```.
Launch,
```
source devel/setup.bash
roslaunch rosbag_exchange vins.launch 
```
The output data should be in such structure,
```
|--SCENE_DIR
    |--rgb
        |--frame_xxxx.jpg
        ...
    |--depth
        |--frame_xxxx.png
        ...
    |-- trajectory.log
    |-- data_association.txt
```
