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

<img src="https://github.com/tanujthakkar/Turtlebot-Path-Planning-ROS/blob/master/Phase_2/Part_1/Results/result.gif"/>


# Turtlebot-Path-Planning-ROS  Phase 2 (Part 2)
## Run Instructions

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin build
source devel/setup.bash

roslaunch astar_turtlebot astar_turtlebot.launch x_init:=5 y_init:=2 theta_init:=0 x_final:=9 y_final:=7 rpm1:=5 rpm2:=10
```