# navlab_turtlebot

ROS workspace for turtlebot autonomy. Developed and tested with Python 3 and ROS Noetic/Melodic

## Setup
1. Install ROS.
2. Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
3. Install python dependencies: `pip3 install --user empy catkin-pkg rospkg`.
4. Install ROS dependencies: `sudo apt-get install ros-"${ROS_DISTRO}"-dwa-local-planner ros-"${ROS_DISTRO}"-turtlebot3 ros-"${ROS_DISTRO}"-vrpn-client-ros ros-"${ROS_DISTRO}"-move-base`
5. In `src/`, clone https://github.com/Stanford-NavLab/navlab_turtlebot_sim. 
4. In `src/mavlab_turtlebot_sim/launch/move_base.launch`, modify line 11 from `<param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />` to `<param name="base_local_planner" value="local_planner/LocalPlanner" />`.
5. Set turtlebot model environment variable in `bashrc`: `export TURTLEBOT3_MODEL="burger"` 
6. Run `catkin build` from workspace root.

