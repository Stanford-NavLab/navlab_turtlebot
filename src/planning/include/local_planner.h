#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <base_local_planner/local_planner_util.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

namespace local_planner{
    /**
     * @class LocalPlanner
     * @brief A class implementing a local planner for the robot
    */
    class LocalPlanner : public nav_core::BaseLocalPlanner{
        public:
            /**
             * @brief  Constructor for the local planner
            */
            LocalPlanner();
            LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);

            /**
             * @brief  Destructor for the local planner
            */
            ~LocalPlanner();

            /** 
             * @brief  Initialization function for the local planner
             * @param name The name of the local planner
             * @param tf A pointer to a transform listener
             * @param costmap_ros The cost map to use for assigning costs to trajectories
            */
            void initialize(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros);

            /**
             * @brief  Set the plan that the controller is following
             * @param orig_global_plan The plan to pass to the controller
             * @return True if the plan was updated successfully, false otherwise
            */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

            /**
             * @brief  Given the current position, orientation, and velocity of the robot,
             * compute velocity commands to send to the base
             * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
             * @return True if a valid trajectory was found, false otherwise
            */
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            /**
             * @brief  Check if the goal pose has been achieved
             * @return True if achieved, false otherwise
            */
            bool isGoalReached();

        private:
        
            /** 
             * @brief  Callback for RTD cmd
             * @param msg RTD twist cmd msg
            */
            void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
            geometry_msgs::Twist cmd_vel_;

            base_local_planner::LocalPlannerUtil planner_util_;   
            
            // Publish global plan to RTD planner, receive back twist commands
            ros::Publisher plan_pub_;
            ros::Subscriber twist_sub_;

            costmap_2d::Costmap2DROS* costmap_ros_;
            tf2_ros::Buffer* tf_;
            bool initialized_;
    };
};

#endif