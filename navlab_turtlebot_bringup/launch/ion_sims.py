import roslaunch
import rospy

# Arguments/Parameters
n_bots = 2
rospy.set_param('n_bots', n_bots)
n_sims = 2

# Initialize things for launching launch file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
cli_args = ['/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_bringup/launch/navigate_multi.launch', \
            'robot_count:=2', \
            'planner:=goal', \
            'goal_file:=/home/izzie/catkin_ws/src/navlab_turtlebot_base/navlab_turtlebot_sim/param/two_swap.yaml']
roslaunch_args = cli_args[1:]
nm_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

# Initialize things for individual nodes
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

# Simulation loop
for sim in range(n_sims):
    launch_nm = roslaunch.parent.ROSLaunchParent(uuid, nm_file)
    launch_nm.start()
    rospy.loginfo("started")
    for i in range(n_bots):
        node = roslaunch.core.Node("navlab_turtlebot_frs", "frs_generator.py", args="__name:=turtlebot"+str(n_bots))
        process = launch.launch(node)
    #node = roslaunch.core.Node("navlab_turtlebot_obstacles", "ground_truth.py")
    #process = launch.launch(node)
    #print(process.is_alive())
    for i in range(30):
        print(i)
        rospy.sleep(1)
    #process.stop()
    launch_nm.shutdown()
