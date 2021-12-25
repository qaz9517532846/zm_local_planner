#include <zm_local_planner/zm_local_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(zm_local_planner::ZMLocalPlanner, nav_core::BaseLocalPlanner)

namespace zm_local_planner
{
    ZMLocalPlanner::ZMLocalPlanner() : tf_(NULL),
                                       state_(Finished),
                                       curr_heading_index_(0),
									   next_heading_index_(0),
									   use_BackForward(false)
    {

    }

    ZMLocalPlanner::~ZMLocalPlanner()
    {
        delete dsrv_;
    }

    void ZMLocalPlanner::reconfigureCB(ZMLocalPlannerConfig &config, uint32_t level)
    {
        ROS_INFO("ZMLocalPlanner reconfigureCB");

        map_frame_ = config.map_frame;
        heading_lookahead_ = config.heading_lookahead;
		start_angle_ = config.start_angle;
        x_vel_.max_vel = config.max_x_vel;
        x_vel_.min_vel = config.min_x_vel;
		x_vel_.limit_acc = config.acc_x_vel;
		y_vel_.max_vel = config.max_y_vel;
        y_vel_.min_vel = config.min_y_vel;
	    y_vel_.limit_acc = config.acc_y_vel;
        rotation_vel_.max_vel = config.max_vel_theta;
        rotation_vel_.min_vel = config.min_vel_theta;
		rotation_vel_.limit_acc = config.acc_theta_vel;
        xy_tolerance_ = config.xy_goal_tolerance;
        yaw_tolerance_ = config.yaw_goal_tolerance;
		yaw_moving_tolerance_ = config.yaw_moving_tolerance;
        transform_timeout_ = config.timeout;
    }

    void ZMLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

        tf_ = tf;
        costmap_ros_ = costmap_ros;

		x_vel_.current_vel = 0;
		y_vel_.current_vel = 0;
		rotation_vel_.current_vel = 0;
		
		ros::NodeHandle global_node;
		next_heading_pub_ = private_nh.advertise<visualization_msgs::Marker>("marker", 10);

        dsrv_ = new dynamic_reconfigure::Server<ZMLocalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<ZMLocalPlannerConfig>::CallbackType cb = boost::bind(&ZMLocalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
        ROS_INFO("ZMLocalPlanner initialized");
    }

    bool ZMLocalPlanner::isGoalReached()
    {
        return (state_ == Finished);
    }

	bool ZMLocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& global_plan)
	{
		last_time_ = ros::Time::now();
		global_plan_.clear();

		// Make our copy of the global plan
		global_plan_ = global_plan;

		ROS_INFO_STREAM("Global plan size: " << global_plan_.size());
        ROS_INFO("Got Plan.");

		curr_heading_index_ = 0;
		next_heading_index_ = 0;
		path_index_ = global_plan_.size() - 1;

		if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}

		// Calculate the rotation between the current odom and the vector created above
		//double rotation = calDeltaAngle(robot_pose_, global_plan_[path_index_]);
		//rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

		double rotation = calStartDeltaAngleOnmi(robot_pose_, start_angle_);

		if(linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) <= xy_tolerance_)
		{
			state_ = RotatingToGoal;
		}
		else if(fabs(rotation) <= yaw_moving_tolerance_ &&
		        linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) > xy_tolerance_)
		{
			state_ = Moving;
		}
		else if(fabs(rotation) > yaw_moving_tolerance_ &&
		        linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) > xy_tolerance_)
		{
			// Set the state to RotatingToStart
			state_ = RotatingToStart;
		}

        nav_msgs::Path global_path_;
        global_path_.header.stamp = ros::Time::now();
        global_path_.header.frame_id = map_frame_;
        global_path_.poses = global_plan_;
        global_plan_pub_.publish(global_path_);
		return true;
	}

	bool ZMLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		// Set all values of cmd_vel to zero
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;

		cmd_vel.angular.x = 0.0; 
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

		// Set the default return value as false
		bool ret = false;

		if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}

		// We need to compute the next heading point from the global plan
		computeNextHeadingIndex();

		switch(state_)
		{
			case RotatingToStart:
			   ret = rotateToStart(cmd_vel);
			   break;
		    case Moving:
			   ret = move(cmd_vel);
			   break;
		    case RotatingToGoal:
			   ret = rotateToGoal(cmd_vel);
			   break;
		    default:
			   return true;
		}

		return ret;
	}

	void ZMLocalPlanner::publishNextHeading(bool show)
	{
		const geometry_msgs::PoseStamped& next_pose = global_plan_[next_heading_index_];

		visualization_msgs::Marker marker;
		marker.id = 0;
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id= next_pose.header.frame_id;
		marker.ns = "waypoints";
		marker.type = visualization_msgs::Marker::CYLINDER;

		if(show)
		{
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose = next_pose.pose;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.2;
			marker.color.a = 0.5;
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		else
		{
			marker.action = visualization_msgs::Marker::DELETE;
		}
		next_heading_pub_.publish(marker);
	}

	bool ZMLocalPlanner::rotateToStart(geometry_msgs::Twist& cmd_vel)
	{
		geometry_msgs::PoseStamped rotate_goal;

		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], rotate_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}

	    //double rotation = calDeltaAngle(robot_pose_, rotate_goal);
		//rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);
		double rotation = calStartDeltaAngleOnmi(robot_pose_, start_angle_);

		if(fabs(rotation) < yaw_moving_tolerance_)
		{
			state_ = Moving;
			return true;
		}

		cmd_vel.angular.z = calRotationVel(rotation);

		return true;
	}

	bool ZMLocalPlanner::move(geometry_msgs::Twist& cmd_vel)
	{
		publishNextHeading();

		geometry_msgs::PoseStamped move_goal;
		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], move_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}
	    
		//double rotation = calDeltaAngle(robot_pose_, move_goal);
		//rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);
		double rotation = calStartDeltaAngleOnmi(robot_pose_, start_angle_);

		cmd_vel.angular.z = calRotationVel(rotation);

		if(fabs(rotation) <= yaw_moving_tolerance_)
		{
			// The robot has rotated to its next heading pose
			cmd_vel.angular.z = 0.0;
		}

		double distance = linearDistance(robot_pose_.pose.position, global_plan_[next_heading_index_].pose.position);
		double angle = calDeltaAngle(robot_pose_, global_plan_[next_heading_index_]);
		double cmd_vel_x = distance * cos(angle) >= 0 ? 
		                   sqrt(2 * x_vel_.limit_acc * distance * fabs(cos(angle))) : 
						   -sqrt(2 * x_vel_.limit_acc * distance * fabs(cos(angle)));
		double cmd_vel_y = distance * sin(angle) >= 0 ? 
		                   sqrt(2 * y_vel_.limit_acc * distance * fabs(sin(angle))) : 
						   -sqrt(2 * y_vel_.limit_acc * distance * fabs(sin(angle)));

		cmd_vel.linear.x = constraint_Onmi(cmd_vel_x, x_vel_);
		cmd_vel.linear.y = constraint_Onmi(cmd_vel_y, y_vel_);

		// The distance from the robot's current pose to the next heading pose
		double distance_to_next_heading = linearDistance(robot_pose_.pose.position, move_goal.pose.position);

		// We are approaching the goal position, slow down
		if(next_heading_index_ == (int)global_plan_.size() - 1)
		{
			// Reached the goal, now we can stop and rotate the robot to the goal position
			if(fabs(distance_to_next_heading) <= xy_tolerance_)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				state_ = RotatingToGoal;
				return true;
			}
		}
		return true;
	}

	bool ZMLocalPlanner::rotateToGoal(geometry_msgs::Twist& cmd_vel)
	{
		geometry_msgs::PoseStamped rotate_goal;

		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], rotate_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}

		double rotation = rewrapAngleRestricted(tf2::getYaw(rotate_goal.pose.orientation) - tf2::getYaw(robot_pose_.pose.orientation));

		if(fabs(rotation) <= yaw_tolerance_)
		{
			state_ = Finished;
			ROS_INFO("Goal reached");
			return true;
		}

		cmd_vel.angular.z = calRotationVel(rotation);

		return true;
	}

	void ZMLocalPlanner::computeNextHeadingIndex()
	{
		geometry_msgs::PoseStamped next_heading_pose;

		for(unsigned int i = curr_heading_index_; i < global_plan_.size() - 1; ++i)
		{
			ros::Time now = ros::Time::now();
			global_plan_[i].header.stamp = now;

			try
			{
				geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      			tf2::doTransform(global_plan_[next_heading_index_], next_heading_pose, trans);
			}
			catch(tf2::LookupException& ex)
			{
				ROS_ERROR("Lookup Error: %s\n", ex.what());
				return;
			}
			catch(tf2::ConnectivityException& ex)
			{
				ROS_ERROR("Connectivity Error: %s\n", ex.what());
				return;
			}
			catch(tf2::ExtrapolationException& ex)
			{
				ROS_ERROR("Extrapolation Error: %s\n", ex.what());
				return;
			}

			double dist = linearDistance(robot_pose_.pose.position, next_heading_pose.pose.position);

			if(dist > heading_lookahead_)
			{
				next_heading_index_ = i;
				return;
			}
			else
			{
				curr_heading_index_++;
			}
		}
		next_heading_index_ = global_plan_.size() - 1;
	}

	double ZMLocalPlanner::calRotationVel(double rotation)
	{
		double vel = 0.0;
		double dt = (ros::Time::now() - last_time_).toSec();
		vel = rotation >= 0 ? sqrt(2 * rotation_vel_.limit_acc * fabs(rotation)) : -sqrt(2 * rotation_vel_.limit_acc * fabs(rotation));

		if(vel > rotation_vel_.current_vel)
		{
			rotation_vel_.current_vel += fmin(vel - rotation_vel_.current_vel, rotation_vel_.limit_acc * dt);
		}
		else
		{
			rotation_vel_.current_vel += fmax(vel - rotation_vel_.current_vel, -rotation_vel_.limit_acc * dt);
		}

		if(rotation_vel_.current_vel > rotation_vel_.max_vel)
		{
			rotation_vel_.current_vel = rotation_vel_.max_vel;
		}
		else if(rotation_vel_.current_vel < rotation_vel_.min_vel)
		{
			rotation_vel_.current_vel = rotation_vel_.min_vel;
		}

		return rotation_vel_.current_vel;
	}

	double ZMLocalPlanner::linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
	{
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}

	double ZMLocalPlanner::calDeltaAngle(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
	{
		double rotation_map;
		double delta_x = p2.pose.position.x - p1.pose.position.x;
		double delta_y = p2.pose.position.y - p1.pose.position.y;

		rotation_map = ::atan2(delta_y, delta_x) - tf2::getYaw(p1.pose.orientation);

		return rewrapAngleRestricted(rotation_map);
	}

	double ZMLocalPlanner::calStartDeltaAngleOnmi(geometry_msgs::PoseStamped p1, double angle)
	{
		double rotation_map = angle - tf2::getYaw(p1.pose.orientation);
		return rewrapAngleRestricted(rotation_map);
	}

	const inline double ZMLocalPlanner::rewrapAngleRestricted(const double angle)
	{
		double rewrap_angle;

		if(angle > PI)
		{
			rewrap_angle = angle - 2 * PI;
		}
		else if (angle < -PI)
		{
			rewrap_angle = angle + 2 * PI;
		}
		else
		{
			rewrap_angle = angle;
		}

		return rewrap_angle;
	}

	const inline double ZMLocalPlanner::RestrictedForwardAngle(const double angle)
	{
		if(angle > PI / 2)
		{
			use_BackForward = true;
			return angle - PI;
		}
		else if(angle < -PI / 2)
		{
			use_BackForward = true;
			return angle + PI;
		}
		else
		{
			use_BackForward = false;
			return angle;
		}
	}

	double ZMLocalPlanner::constraint_Onmi(double vel, constraint_vel &set)
	{
		double dt = (ros::Time::now() - last_time_).toSec();
		if(vel > set.current_vel)
		{
			set.current_vel += fmin(vel - set.current_vel, set.limit_acc * dt);
		}
		else
		{
			set.current_vel += fmax(vel - set.current_vel, -set.limit_acc * dt);
		}

		if(set.current_vel > set.max_vel)
		{
			set.current_vel = set.max_vel;
		}
		else if (set.current_vel < set.min_vel)
		{
			set.current_vel = set.min_vel;
		}

		//set.last_vel = velocity;
		//ROS_INFO("vel = %f", set.current_vel);
		
		return set.current_vel;
	}
}
