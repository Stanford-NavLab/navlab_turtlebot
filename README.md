# navlab_turtlebot

ROS workspace for turtlebot autonomy. Developed and tested with Python 3 and ROS Noetic/Melodic

## Setup
1. Install ROS.
2. Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
3. Install python dependencies:
```
pip3 install --user empy catkin-pkg rospkg
```
4. Install ROS dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-turtlebot3 ros-"${ROS_DISTRO}"-move-base ros-"${ROS_DISTRO}"-teb-local-planner
```
5. Create a ROS workspace and clone this repo into the `src/` folder of your workspace.
    * If base station, additionally (also in `src/`), clone https://github.com/Stanford-NavLab/navlab_turtlebot_base and install dependencies.
7. Run `catkin build` from workspace root.

## Run
Gazebo two robot position exchange
1. `roslaunch navlab_turtlebot_bringup navigate_multi.launch`
  - `robot_count` = 4
  - `goal_file` = `two_swap.yaml`
2. Start the controllers
  - `rosrun navlab_turtlebot_control twist_sender.py -n turtlebot1`
  - `rosrun navlab_turtlebot_control twist_sender.py -n turtlebot2`
3. Start the planners  
  - `rosrun navlab_turtlebot_planning dubins_planner.py -n turtlebot1`
  - `rosrun navlab_turtlebot_planning dubins_planner.py -n turtlebot2`
---
In general:
`roslaunch navlab_turtlebot_bringup navigate_multi.launch`

args: 
 - `robot_count:=n`
 - `sim:=true/false`
 - `planner:="" or "goal"`
 - `goal_file:=<filename>.yaml`
   
(Params/goal file location: `navlab_turtlebot_base/navlab_turtlebot_sim/params/`)
 - For now, easiest to edit `navigate_multi.launch` to adjust params file

Change ROS planner: `navlab_turtlebot/navlab_turtlebot_planning/launch/move_base_multi.launch`
 - TEB planner or DWA planner
 - Also edit planner parameters here

TEB planner obstacles:
1. publish to topic: `/turtlebot1/move_base/TebLocalPlannerROS/obstacles`

## Troubleshooting
If you get an error that says:
```
/usr/bin/env: ‘python’: No such file or directory
```
Then you may need to add a symlink so ROS can find your Python version. For example
```
sudo ln -s /usr/bin/python3 /usr/bin/python
```
