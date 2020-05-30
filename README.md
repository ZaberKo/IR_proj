roslaunch pokemon multi_turtlebot3.launch

roslaunch turtlebot3_navigation multi_nav_bringup.launch

rosrun pokemon server.py

rosrun pokemon auto_capture.py 0

rosrun pokemon auto_capture.py 1

rosrun pokemon auto_capture.py 2


launch/multi_turtlebot3.launch 里world_name地址需要更改.

maps/final_pro2.yaml 里image地址需要更改.

