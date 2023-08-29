import roslaunch
import rospy
import numpy as np

# Arguments/Parameters
n_bots = 2
rospy.set_param('n_bots', n_bots)
n_sims = 1
sim_len = 60

# Initialize things for launching launch file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
cli_args = ['/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_bringup/launch/navigate_multi.launch', \
            'robot_count:=2', \
            'planner:=goal', \
            'goal_file:=/home/izzie/catkin_ws/src/navlab_turtlebot_base/navlab_turtlebot_frs/param/current_goal.yaml']
roslaunch_args = cli_args[1:]
nm_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
ion_args = ['/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_bringup/launch/ion-stuff.launch', \
            'name:=turtlebot1']
roslaunch_ionargs = ion_args[1:]
ion_file = [(roslaunch.rlutil.resolve_launch_arguments(ion_args)[0], roslaunch_ionargs)]

# Initialize things for individual nodes
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

# Simulation loop
for sim in range(n_sims):
    rospy.loginfo("STARTING SIM #" + str(sim))
    
    with open("current_goal.yaml","w") as f:
        for bot in range(n_bots):
            f.write("turtlebot"+str(bot+1)+":")
            # in outer edge of 6x6 square with edge on y axis
            rndxy = np.random.rand(2,2)*6
            rndyaw = np.random.rand(2)*2*np.pi
            f.write("  start_x: "+rndvals[0,0])
            f.write("  start_y: "+rndvals[0,1])
            f.write("  start_yaw: "+rndyaw[0])
            f.write("  goal_x: "+rndvals[0,0])
            f.write("  goal_y: "+rndvals[0,1])
            f.write("  goal_yaw: "+rndyaw[1])
    
    # Launch frs-related files
    launch_ion = roslaunch.parent.ROSLaunchParent(uuid, ion_file)
    launch_ion.start()
    
    # Launch simulator
    launch_nm = roslaunch.parent.ROSLaunchParent(uuid, nm_file)
    launch_nm.start()

    for i in range(sim_len):
        rospy.sleep(1)
        
    # Close everything
    launch_ion.shutdown()
    launch_nm.shutdown()
