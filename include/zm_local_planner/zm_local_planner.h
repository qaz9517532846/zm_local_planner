#ifndef ZM_LOCAL_PLANNER_H_
#define ZM_LOCAL_PLANNER_H_

#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <zm_local_planner/ZMLocalPlannerConfig.h>

#define PI 3.1415926543

namespace zm_local_planner
{
    class ZMLocalPlanner : public nav_core::BaseLocalPlanner
    {
        public:
           ZMLocalPlanner();
           ~ZMLocalPlanner();

           void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

           bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
           bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
           bool isGoalReached();

        private:
           void publishNextHeading(bool show = true);
           bool rotateToStart(geometry_msgs::Twist& cmd_vel);
           bool move(geometry_msgs::Twist& cmd_vel);
           bool rotateToGoal(geometry_msgs::Twist& cmd_vel);
           void computeNextHeadingIndex();
           double calLinearVel();
           double calRotationVel(double rotation);
           double linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
           double mapToMinusPIToPI(double angle);
           void reconfigureCB(ZMLocalPlannerConfig &config, uint32_t level);

	       dynamic_reconfigure::Server<ZMLocalPlannerConfig> *dsrv_;
           
           typedef enum
           {
               RotatingToStart,
               Moving,
               RotatingToGoal,
               Finished
           } State;

           State state_;
            
           tf2_ros::Buffer* tf_;
           costmap_2d::Costmap2DROS* costmap_ros_;
           std::vector<geometry_msgs::PoseStamped> global_plan_;
            
           geometry_msgs::PoseStamped robot_pose_;
           ros::Publisher next_heading_pub_;

           int curr_heading_index_, next_heading_index_;
           int path_index_;

           ros::Publisher global_plan_pub_;
            
           // Parameters
           std::string map_frame_;

           struct constraint_vel
           {
               double max_vel;
               double min_vel;
           };

           constraint_vel linear_vel_, rotation_vel_;
           
           double heading_lookahead_;
           double yaw_tolerance_, xy_tolerance_;
           double yaw_moving_tolerance_;
           double transform_timeout_;
    };

};

#endif