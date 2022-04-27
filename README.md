# ENPM661 Project 3

Tanuj Thakkar (UID - 117817539)
Naitri Rajyaguru (UID - 117361919)

# Turtlebot-Path-Planning-ROS  Phase 1
## Run Instructions

```
python3.8 AStar.py --StartState [16,16,30] --GoalState [350,234,0] --StepSize 10 --Threshold 5 --Visualize
```

## Astar Output


<img src="https://github.com/tanujthakkar/Turtlebot-Path-Planning-ROS/blob/master/Phase_1/Results/video.gif"/>

# Turtlebot-Path-Planning-ROS  Phase 2 (Part 1)
## Run Instructions

```
python3.8 AStar.py --start "1,1,30" --goal "9,9,0" --rpm "50,100" --clearance 0.1 --save_path "../Results/"
```

<img src="https://github.com/tanujthakkar/Turtlebot-Path-Planning-ROS/blob/master/Phase_2/Part_1/Results/out.gif"/>

# Turtlebot-Path-Planning-ROS  Phase 2 (Part 2)
## Run Instructions

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
source devel/setup.bash

roslaunch astar_turtlebot astar_turtlebot.launch sx:=5 sy:=2 syaw:=0 rpm1:=5 rpm2:=10 clearance:=5

After RViz and Gazebo load up, pass the goal point in RViz map using the "2D Nav Goal" button.
```

Please reach out to me if you face any issues running the code.
Tanuj Thakkar - tanuj@umd.edu
Naitri Rajyaguru - nrajyagu@umd.edu