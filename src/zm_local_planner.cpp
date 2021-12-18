#include <zm_local_planner/zm_local_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(zm_local_planner::ZMLocalPlanner, nav_core::BaseLocalPlanner)

namespace zm_local_planner
{
    ZMLocalPlanner::ZMLocalPlanner() : tf_(NULL),
                                       state_(Finished),
                                       curr_heading_index_(0),
									   next_heading_index_(0)
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
        linear_vel_.max_vel = config.max_linear_vel;
        linear_vel_.min_vel = config.min_linear_vel;
        rotation_vel_.max_vel = config.max_vel_theta;
        rotation_vel_.min_vel = config.min_vel_theta;
        xy_tolerance_ = config.xy_goal_tolerance;
        yaw_tolerance_ = config.yaw_goal_tolerance;
        transform_timeout_ = config.timeout;
    }

    void ZMLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

        tf_ = tf;
        costmap_ros_ = costmap_ros;
		
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

		// Create a vector between the current odom pose to the next heading pose
		double x = global_plan_[path_index_].pose.position.x - robot_pose_.pose.position.x;
		double y = global_plan_[path_index_].pose.position.y - robot_pose_.pose.position.y;

		// Calculate the rotation between the current odom and the vector created above
		double rotation = (::atan2(y,x) - tf2::getYaw(robot_pose_.pose.orientation));
		rotation = mapToMinusPIToPI(rotation);

		if(linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) <= xy_tolerance_)
		{
			state_ = RotatingToGoal;
		}
		else if(rotation <= yaw_tolerance_ && path_index_ < 10)
		{
			state_ = Moving;
		}
		else if(path_index_ > 10)
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

		// Create a vector between the current odom pose to the next heading pose
		double x = rotate_goal.pose.position.x - robot_pose_.pose.position.x;
		double y = rotate_goal.pose.position.y - robot_pose_.pose.position.y;

		// Calculate the rotation between the current odom and the vector created above
		double rotation = (::atan2(y,x) - tf2::getYaw(robot_pose_.pose.orientation));

		rotation = mapToMinusPIToPI(rotation);

		if(fabs(rotation) < yaw_tolerance_)
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

		// Create a vector between the current odom pose to the next heading pose
		double x = move_goal.pose.position.x - robot_pose_.pose.position.x;
		double y = move_goal.pose.position.y - robot_pose_.pose.position.y;

		// Calculate the rotation between the current odom and the vector created above
		double rotation = (::atan2(y,x) - tf2::getYaw(robot_pose_.pose.orientation));

		rotation = mapToMinusPIToPI(rotation);

		cmd_vel.angular.z = calRotationVel(rotation);

		if(fabs(rotation) <= yaw_tolerance_)
		{
			// The robot has rotated to its next heading pose
			cmd_vel.angular.z = 0.0;
		}

		cmd_vel.linear.x = calLinearVel();

		// The distance from the robot's current pose to the next heading pose
		double distance_to_next_heading = linearDistance(robot_pose_.pose.position, move_goal.pose.position);

		// We are approaching the goal position, slow down
		if(next_heading_index_ == (int) global_plan_.size() - 1)
		{
			// Reached the goal, now we can stop and rotate the robot to the goal position
			if(distance_to_next_heading <= xy_tolerance_)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				state_ = RotatingToGoal;
				return true;
			}
		}
		return true;
	}

	bool ZMLocalPlanner::rotateToGoal( geometry_msgs::Twist& cmd_vel )
	{
		geometry_msgs::PoseStamped rotate_goal;

		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], rotate_goal, trans);

			// tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
			// tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], rotate_goal );
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

		double rotation = tf2::getYaw( rotate_goal.pose.orientation ) -
				tf2::getYaw(robot_pose_.pose.orientation );

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

				// tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[i].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
				// tf_->transformPose( base_odom_.header.frame_id, global_plan_[i], next_heading_pose );
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

	double ZMLocalPlanner::calLinearVel()
	{
		double vel = 0.0;

	    double straight_dist = linearDistance(robot_pose_.pose.position, global_plan_[next_heading_index_].pose.position);
		vel = straight_dist;

		if(vel > linear_vel_.max_vel)
		   vel = linear_vel_.max_vel;

		if(vel < linear_vel_.min_vel)
		   vel = linear_vel_.min_vel;

		return vel;
	}

	double ZMLocalPlanner::calRotationVel(double rotation)
	{
		double vel = 0.0;

		if(rotation > rotation_vel_.max_vel)
		{
			vel = rotation_vel_.max_vel;
		}
		else if(rotation < rotation_vel_.min_vel)
		{
			vel = rotation_vel_.min_vel;
		}
		else
		{
			vel = rotation;
		}

		return vel;
	}

	double ZMLocalPlanner::linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
	{
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}

	double ZMLocalPlanner::mapToMinusPIToPI(double angle)
	{
		double angle_overflow = static_cast<double>(static_cast<int>(angle / PI));

		if(angle_overflow > 0.0)
		{
			angle_overflow = ceil(angle_overflow / 2.0);
		}
		else
		{
			angle_overflow = floor(angle_overflow / 2.0);
		}

		angle -= 2 * PI * angle_overflow;
		return angle;
	}
}