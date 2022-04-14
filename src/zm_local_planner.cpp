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
        global_lookahead_ = config.global_lookahead;
		local_lookahead_ = config.local_lookahead;

        linear_vel_.max_vel = config.max_linear_vel;
        linear_vel_.min_vel = config.min_linear_vel;
		linear_vel_.limit_acc = config.acc_linear_vel;
        rotation_vel_.max_vel = config.max_vel_theta;
        rotation_vel_.min_vel = config.min_vel_theta;
		rotation_vel_.limit_acc = config.acc_vel_theta;

        xy_tolerance_ = config.xy_goal_tolerance;
        yaw_tolerance_ = config.yaw_goal_tolerance;
		yaw_moving_tolerance_ = config.yaw_moving_tolerance;

		obstacle_cost_ = config.obstacle_cost;
        avoid_offset_x_ = config.avoid_offset_x;
        avoid_offset_y_ = config.avoid_offset_y;

        transform_timeout_ = config.timeout;
    }

    void ZMLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

        tf_ = tf;
        costmap_ros_ = costmap_ros;

		linear_vel_.current_vel = 0;
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

	bool ZMLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
	{
		last_time_ = ros::Time::now();
		global_plan_.clear();
		local_plan_.clear();

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

		// We need to compute the next heading point from the global plan
		//computeNextHeadingIndex(global_plan_, next_heading_index_);

		// Calculate the rotation between the current odom and the vector created above
		double rotation = calDeltaAngle(robot_pose_, global_plan_[path_index_]);
		rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

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

		nav_msgs::Path global_path_ = path_publisher("map", global_plan_);
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

		// calculate local plan from global plan and local cost map.
		local_plan_ = cal_local_planner(costmap_ros_, robot_pose_, global_plan_);
		computeNextHeadingIndex(local_plan_, next_heading_index_);
		//ROS_INFO("next_heading_index = %d", next_heading_index_);
		nav_msgs::Path local_path_ = path_publisher("map", local_plan_);
		local_plan_pub_.publish(local_path_);

		if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}

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
		const geometry_msgs::PoseStamped& next_pose = local_plan_[next_heading_index_];

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
		local_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, local_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(local_plan_[next_heading_index_], rotate_goal, trans);
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

	    double rotation = calDeltaAngle(robot_pose_, rotate_goal);
		rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

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
		local_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, local_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(local_plan_[next_heading_index_], move_goal, trans);
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
	    
		double rotation = calDeltaAngle(robot_pose_, move_goal);
		rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

		cmd_vel.angular.z = calRotationVel(rotation);

		if(fabs(rotation) <= yaw_moving_tolerance_)
		{
			// The robot has rotated to its next heading pose
			cmd_vel.angular.z = 0.0;
		}

		cmd_vel.linear.x = calLinearVel();

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
		local_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, local_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(local_plan_[next_heading_index_], rotate_goal, trans);
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

	void ZMLocalPlanner::computeNextHeadingIndex(std::vector<geometry_msgs::PoseStamped> plan, int& cal_next_index_)
	{
		geometry_msgs::PoseStamped next_heading_pose;
		cal_next_index_ = 0;

		for(unsigned int i = curr_heading_index_; i < plan.size() - 1; ++i)
		{
			ros::Time now = ros::Time::now();
			plan[i].header.stamp = now;

			try
			{
				geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, plan[i].header.frame_id, now, ros::Duration(transform_timeout_));
      			tf2::doTransform(plan[i], next_heading_pose, trans);
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
			cal_next_index_ = i;
			if(dist > heading_lookahead_)
			{
				break;
			}
		}
	}

	double ZMLocalPlanner::calLinearVel()
	{
		double vel = 0.0;

	    double straight_dist = linearDistance(robot_pose_.pose.position, local_plan_[next_heading_index_].pose.position);

		vel = use_BackForward == false ? 
		                         sqrt(2 * linear_vel_.limit_acc * straight_dist) :
								 -sqrt(2 * linear_vel_.limit_acc * straight_dist);

		double dt = (ros::Time::now() - last_time_).toSec();

		if(vel > linear_vel_.current_vel)
		{
			linear_vel_.current_vel += fmin(vel - linear_vel_.current_vel, linear_vel_.limit_acc * dt);
		}
		else
		{
			linear_vel_.current_vel += fmax(vel - linear_vel_.current_vel, -linear_vel_.limit_acc * dt);
		}

		if(linear_vel_.current_vel > linear_vel_.max_vel)
		{
			linear_vel_.current_vel = linear_vel_.max_vel;
		}
		else if (linear_vel_.current_vel < linear_vel_.min_vel)
		{
			linear_vel_.current_vel = linear_vel_.min_vel;
		}

		return linear_vel_.current_vel;
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

	std::vector<geometry_msgs::Point> ZMLocalPlanner::get_footprint_cost(costmap_2d::Costmap2DROS* costmap_ros, geometry_msgs::PoseStamped pose)
	{
		std::vector<geometry_msgs::Point> footprint_position = costmap_ros->getRobotFootprint();
		for(int i = 0; i < footprint_position.size(); i++)
		{
			tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
			auto bound_pos = tf2::Matrix3x3(q) * tf2::Vector3(footprint_position[i].x, footprint_position[i].y, 0);
			footprint_position[i].x = pose.pose.position.x + bound_pos[0];
			footprint_position[i].y = pose.pose.position.y + bound_pos[1];
		}

		return footprint_position;
	}

	int ZMLocalPlanner::get_cost(costmap_2d::Costmap2DROS* costmap_ros, geometry_msgs::Point pose)
	{
		int local_costmap_pos[2];
		costmap_ros->getCostmap()->worldToMapEnforceBounds(pose.x, pose.y, local_costmap_pos[0], local_costmap_pos[1]);
		//ROS_INFO("cost_map_pos_x = %d, cost_map_pos_y = %d", local_costmap_pos[0], local_costmap_pos[1]);
		return costmap_ros->getCostmap()->getCost(local_costmap_pos[0], local_costmap_pos[1]);
	}

	std::vector<geometry_msgs::PoseStamped> ZMLocalPlanner::cal_local_planner(costmap_2d::Costmap2DROS* costmap_ros, geometry_msgs::PoseStamped pose, std::vector<geometry_msgs::PoseStamped> global_plan)
	{
		std::vector<geometry_msgs::PoseStamped> cal_local_plan_;

		bool obstacle_footprint_1 = false;
		bool obstacle_footprint_2 = false;
		bool obstacle_footprint_3 = false;
		bool obstacle_footprint_4 = false;

		bool have_obstacle = false;
		int num_obstacle = 0;

		bool left_obstacle = false;
		bool right_obstacle = false;

		std::vector<geometry_msgs::Point> footprint_pos;
		footprint_pos = get_footprint_cost(costmap_ros, pose);

		int footprint_cost_[4];
		for(int i = 0; i < 4; i++)
		{
			footprint_cost_[i] = get_cost(costmap_ros_, footprint_pos[i]);
		}

		obstacle_footprint_1 = footprint_cost_[0] >= obstacle_cost_ ? true : false;
		obstacle_footprint_2 = footprint_cost_[1] >= obstacle_cost_ ? true : false;
		obstacle_footprint_3 = footprint_cost_[2] >= obstacle_cost_ ? true : false;
		obstacle_footprint_4 = footprint_cost_[3] >= obstacle_cost_ ? true : false;
		have_obstacle = obstacle_footprint_1 || obstacle_footprint_2 || obstacle_footprint_3 || obstacle_footprint_4;
		//ROS_INFO("%d, %d, %d, %d", footprint_cost_[0], footprint_cost_[1], footprint_cost_[2], footprint_cost_[3]);
		//ROS_INFO("have_obstacle = %d", have_obstacle);

		if(have_obstacle)
		{
			heading_lookahead_ = global_lookahead_;
			geometry_msgs::PoseStamped back_pose = pose;
			cal_local_plan_.push_back(pose);
			tf2::Vector3 back_point;
			double back_point_rpy_[3];
			tf2::Quaternion q(pose.pose.orientation.x, 
			                  pose.pose.orientation.y, 
							  pose.pose.orientation.z, 
							  pose.pose.orientation.w);
			
			// 3x3 Rotation matrix from quaternion
			tf2::Matrix3x3 m(q);
			// Roll Pitch and Yaw from rotation matrix
			m.getRPY(back_point_rpy_[0], back_point_rpy_[1], back_point_rpy_[2]);	

			float back_distance_divide = avoid_offset_x_ / 10;
			float back_point_yaw_divide = atan(avoid_offset_y_ / avoid_offset_x_) / 10;
			int yaw_direction = 1;

			double back_rotation = calDeltaAngle(pose, global_plan[path_index_]);
			//ROS_INFO("Delta Angle = %f", back_rotation);
			if(back_rotation > -PI / 2 && back_rotation < PI / 2)
			{
				if(obstacle_footprint_1 && !obstacle_footprint_2)
				{
					yaw_direction = -1;
				}
				else if(!obstacle_footprint_1 && obstacle_footprint_2)
				{
					yaw_direction = 1;
				}
			}
			else
			{
				if(obstacle_footprint_4 && !obstacle_footprint_3)
				{
					yaw_direction = 1;
				}
				else if(!obstacle_footprint_4 && obstacle_footprint_3)
				{
					yaw_direction = -1;
				}
			}
			
			for(int num = 0; num < 10; num++)
			{
				tf2::Quaternion back_quat;
				geometry_msgs::Quaternion back_quat_tf;
				back_point_rpy_[2] += yaw_direction * back_point_yaw_divide;
				back_quat.setRPY(back_point_rpy_[0], back_point_rpy_[1], back_point_rpy_[2]);
				back_quat_tf = tf2::toMsg(back_quat);
				back_pose.pose.orientation = back_quat_tf;

				back_point = tf2::Matrix3x3(back_quat) * tf2::Vector3(yaw_direction * back_distance_divide, 0, 0);
				back_pose.pose.position.x += back_point[0];
				back_pose.pose.position.y += back_point[1];
				cal_local_plan_.push_back(back_pose);
			}
		}
		else
		{
			heading_lookahead_ = global_lookahead_;
			cal_local_plan_ = global_plan;
		}

		return cal_local_plan_;
	}

	nav_msgs::Path ZMLocalPlanner::path_publisher(std::string frame, std::vector<geometry_msgs::PoseStamped> plan)
	{
		nav_msgs::Path pub_path_;
		pub_path_.header.stamp = ros::Time::now();
        pub_path_.header.frame_id = frame;
        pub_path_.poses = plan;
		return pub_path_;
	}
}