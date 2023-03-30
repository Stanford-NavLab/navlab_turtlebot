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
5. In `src/`, clone https://github.com/Stanford-NavLab/navlab_turtlebot_base.
6. In `src/navlab_turtlebot_base/launch/move_base.launch`, modify line 11 from `<param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />` to `<param name="base_local_planner" value="local_planner/LocalPlanner" />`.
7. Run `catkin build` from workspace root.

## Run
Two robot position exchange
1. `roslaunch navlab_turtlebot_base navigate_multi.launch`
  - `robot_count` = 4
  - `goal_file` = `two_swap.yaml`
2. Start the controllers
  - `rosrun navlab_turtlebot_control twist_sender.py -n turtlebot1`
  - `rosrun navlab_turtlebot_control twist_sender.py -n turtlebot2`
3. Start the planners  
  - `rosrun navlab_turtlebot_planning dubins_planner.py -n turtlebot1`
  - `rosrun navlab_turtlebot_planning dubins_planner.py -n turtlebot2`


## Troubleshooting
If you get an error that says:
```
/usr/bin/env: ‘python’: No such file or directory
```
Then you may need to add a symlink so ROS can find your Python version. For example
```
sudo ln -s /usr/bin/python3 /usr/bin/python
```
