import roslaunch
import rospy
import numpy as np
import csv

# Arguments/Parameters
n_bots = 2
rospy.set_param('n_bots', n_bots)
n_sims = 5
sim_len = 60
rospy.set_param('sim_or_cal','cal')

# Initialize things for launching launch file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
cli_args = ['/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_bringup/launch/navigate_multi.launch', \
            'robot_count:=2', \
            'planner:=goal', \
            'goal_file:=/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/param/current_goal.yaml', \
            'open_rviz:=false']
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
    print("STARTING SIM #" + str(sim))
    
    with open("/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/param/current_goal.yaml","w") as f:
        for bot in range(n_bots):
            f.write("turtlebot"+str(bot+1)+":\n")
            # in 5x5 square with edge on y axis
            rndxy = np.random.rand(2,2)*5-np.array([[5,2.5],[5,2.5]])
            rndyaw = np.random.rand(2)*2*np.pi
            f.write("  start_x: "+str(rndxy[0,0])+"\n")
            f.write("  start_y: "+str(rndxy[0,1])+"\n")
            f.write("  start_yaw: "+str(rndyaw[0])+"\n")
            f.write("  goal_x: "+str(rndxy[1,0])+"\n")
            f.write("  goal_y: "+str(rndxy[1,1])+"\n")
            f.write("  goal_yaw: "+str(rndyaw[1])+"\n")
            rows = [[rndxy[0,0],rndxy[0,1],rndyaw[0],rndxy[1,0],rndxy[1,1],rndyaw[1]]]
            with open("/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/simdeets.csv","a") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)
    
    # Launch simulator
    launch_nm = roslaunch.parent.ROSLaunchParent(uuid, nm_file)
    launch_nm.start()
    
    rospy.set_param('ending',False)
    
    # Launch frs-related files
    launch_ion = roslaunch.parent.ROSLaunchParent(uuid, ion_file)
    launch_ion.start()

    rospy.sleep(sim_len)
    
    print("ending")
    rospy.set_param('ending',True)
    rospy.sleep(1)
        
    #print("Before closing", np.loadtxt('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(0)+'.csv', delimiter=',').shape) 
    #print("Before closing", np.loadtxt('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(1)+'.csv', delimiter=',').shape) 
        
    # Close everything
    launch_ion.shutdown()
    launch_nm.shutdown()
    
    #print("After closing", np.loadtxt('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(0)+'.csv', delimiter=',').shape)
    #print("After closing", np.loadtxt('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(1)+'.csv', delimiter=',').shape)
