import roslaunch
import rospy
import numpy as np
import csv

navlab_turtlebot_dir = "/home/derek/jpl_ws/src/"

# Arguments/Parameters
n_bots = 2
rospy.set_param('n_bots', n_bots)
n_sims = 51
sim_len = 45 # In 100 sims I found a max time of 26.1 seconds, average 12ish.
rospy.set_param('sim_or_cal','cal')
# TEB Local Planner Parameters
for bot in range(n_bots):
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/controller_frequency',20.0)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/planner_frequency',1.0)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/max_planning_retries',50)

    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/NavfnROS/allow_unknown',True)

    mvx = rospy.get_param('~/turtlebot'+str(bot+1)+'/move_base/DWAPlannerROS/max_vel_x')
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/max_vel_x',mvx)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/max_vel_x_backwards',.02)
    mvth = rospy.get_param('~/turtlebot'+str(bot+1)+'/move_base/DWAPlannerROS/max_vel_theta')
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/max_vel_theta',mvth)
    alx = rospy.get_param('~/turtlebot'+str(bot+1)+'/move_base/DWAPlannerROS/acc_lim_x')
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/acc_lim_x',alx)
    alth = rospy.get_param('~/turtlebot'+str(bot+1)+'/move_base/DWAPlannerROS/acc_lim_theta')
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/acc_lim_theta',alth)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/footprint_model/type','circular')
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/footprint_model/radius',.105)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/min_obstacle_dist',.105)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/inflation_dist',.105)
    #rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/global_plan_viapoint_sep',.1)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/penalty_epsilon',.01)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/weight_obstacle',100)

    # trajectory
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist',4.0)
    rospy.set_param('~/turtlebot'+str(bot+1)+'/move_base/TebLocalPlannerROS/feasibility_check_no_poses',10)

# Initialize things for launching launch file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
cli_args = [navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_bringup/launch/navigate_multi.launch', \
            'robot_count:=2', \
            'planner:=goal', \
            'goal_file:=' + navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_frs/param/current_goal.yaml', \
            'open_rviz:=true']
roslaunch_args = cli_args[1:]
nm_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
ion_args = [navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_bringup/launch/ion-stuff.launch', \
            'name:=turtlebot1']
roslaunch_ionargs = ion_args[1:]
ion_file = [(roslaunch.rlutil.resolve_launch_arguments(ion_args)[0], roslaunch_ionargs)]

# Initialize things for individual nodes
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

# Simulation loop
for sim in range(n_sims):
    print("STARTING SIM #" + str(sim))

    #np.random.seed(4)
    with open(navlab_turtlebot_dir + "navlab_turtlebot/navlab_turtlebot_frs/param/current_goal.yaml","w") as f:
        for bot in range(n_bots):
            f.write("turtlebot"+str(bot+1)+":\n")
            print("turtlebot"+str(bot+1)+":\n")
            # in 5x5 square with edge on y axis
            if False: #bot == 0:
                rndxy = np.array([[1,1],[1,1]])
                rndyaw = np.zeros(2)
            else:
                rndxy = np.random.rand(2,2)*5-np.array([[5,2.5],[5,2.5]])
                rndyaw = np.random.rand(2)*2*np.pi
            """if bot == 0:
                rndxy[1,0] = -4
                rndxy[1,1] = 2.5
                rndxy[0,0] = -.25
                rndxy[0,1] = -.25"""
            # start
            f.write("  start_x: "+str(rndxy[0,0])+"\n")
            print("  start_x: "+str(rndxy[0,0])+"\n")
            f.write("  start_y: "+str(rndxy[0,1])+"\n")
            print("  start_y: "+str(rndxy[0,1])+"\n")
            f.write("  start_yaw: "+str(rndyaw[0])+"\n")
            # goals
            f.write("  goal_x: "+str(rndxy[1,0])+"\n")
            print("  goal_x: "+str(rndxy[1,0])+"\n")
            f.write("  goal_y: "+str(rndxy[1,1])+"\n")
            print("  goal_y: "+str(rndxy[1,1])+"\n")
            f.write("  goal_yaw: "+str(rndyaw[1])+"\n")
            rows = [[rndxy[0,0],rndxy[0,1],rndyaw[0],rndxy[1,0],rndxy[1,1],rndyaw[1]]]
            print("goals")
            print(rows)
            print("\n")
            with open(navlab_turtlebot_dir + "navlab_turtlebot/navlab_turtlebot_frs/data/simdeets.csv","a") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)

    rospy.set_param('ending',False)

    # Launch simulator
    launch_nm = roslaunch.parent.ROSLaunchParent(uuid, nm_file)
    launch_nm.start()
    print(rospy.get_param('turtlebot1/start_x'))
    print(rospy.get_param('turtlebot1/start_y'))

    # Launch frs-related files
    launch_ion = roslaunch.parent.ROSLaunchParent(uuid, ion_file)
    launch_ion.start()

    rospy.sleep(sim_len)

    print("Before closing", \
          np.loadtxt(navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(0)+'.csv', delimiter=',').shape, \
          np.loadtxt(navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(1)+'.csv', delimiter=',').shape)

    print("ending")
    rospy.set_param('ending',True)
    rospy.sleep(1)

    # Close everything
    launch_ion.shutdown()
    launch_nm.shutdown()

    print("After closing", \
          np.loadtxt(navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(0)+'.csv', delimiter=',').shape, \
          np.loadtxt(navlab_turtlebot_dir + 'navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(1)+'.csv', delimiter=',').shape)
    print("\n\n\n")
