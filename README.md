# navlab_turtlebot

ROS workspace for turtlebot autonomy. Developed and tested with Python 3 and ROS Noetic/Melodic

## Setup
1. Install ROS.
2. Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
3. Install dependencies: `pip3 install --user empy catkin-pkg rospkg`.
4. Clone [navlab_turtlebot_sim](https://github.com/Stanford-NavLab/navlab_turtlebot_sim) package into `/src` and install dependencies.
5. Set turtlebot model environment variable in `bashrc`: `export TURTLEBOT3_MODEL="burger"` 
6. Run `catkin build` from workspace root.
