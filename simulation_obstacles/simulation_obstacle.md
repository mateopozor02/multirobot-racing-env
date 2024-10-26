1.Run this two files

``` 
roslaunch deepexpress_gazebo simulation.launch 
```
```
rosrun deepexpress_gazebo motion.py
```

insie the  motion.py we can find two topics

1) to set velocity, the boxes use this topic
```
/box1/command_velocity
/box2/command_velocity
```
2) to know the position of the boxes, subscribe this topic
```
/box1_0/odom
/box2_0/odom
```
