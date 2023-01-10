#include "local_planner.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

    LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    LocalPlanner::~LocalPlanner() {}


    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            initialized_ = true;

            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            twist_sub_ = private_nh.subscribe("rtd_cmd", 1, &LocalPlanner::twistCallback, this);
        }
    }


    void LocalPlanner::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        ROS_INFO("Got twist command");
        ROS_INFO("  linear.x: %f", msg->linear.x);
        ROS_INFO("  angular.z: %f", msg->angular.z);
    }


    bool LocalPlanner::setPlan(
        const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
    )
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        ROS_INFO("Got new plan");
        // for (auto pose : orig_global_plan)
        // {
        //     ROS_INFO("  x: %f, y: %f", pose.pose.position.x, pose.pose.position.y);
        // }
        return true;
    }


    bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        ROS_INFO("Computing velocity commands");
        cmd_vel.linear.x = 0.1;
        return true;
    }


    bool LocalPlanner::isGoalReached()
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        return false;
    }
};
