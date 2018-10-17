#include <dwl_msgs/ControllerCommons.h>


namespace dwl_msgs
{

ControllerCommons::ControllerCommons() : init_base_state_(false), new_plan_(false),
		num_traj_points_(0), trajectory_counter_(0), controller_publish_rate_(250),
		robot_publish_rate_(250), odom_publish_rate_(250), imu_publish_rate_(250),
		init_controller_state_pub_(false), init_robot_state_pub_(false),
		init_odom_state_pub_(false), init_imu_state_pub_(false)
{

}


ControllerCommons::~ControllerCommons()
{

}


void ControllerCommons::initControllerStatePublisher(ros::NodeHandle node,
													 const dwl::model::FloatingBaseSystem& fbs)
{
	// Getting the publish period
	if (!node.getParam("publish_rate", controller_publish_rate_)) {
		ROS_WARN("Parameter of ControllerState publisher 'publish_rate' not set"
				" (default value is 250). It has to be defined in"
				" %s/publish_rate", node.getNamespace().c_str());
	}

	// Setting the floating-base system info
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
	wb_iface_.reset(fbs);

	// Initializing the real-time publisher
	controller_state_pub_.reset(new
			realtime_tools::RealtimePublisher<dwl_msgs::WholeBodyController> (node, "state", 1));
	controller_state_pub_->lock();

	controller_state_pub_->msg_.desired.joints.resize(fbs_->getJointDoF());
	controller_state_pub_->msg_.actual.joints.resize(fbs_->getJointDoF());
	controller_state_pub_->msg_.error.joints.resize(fbs_->getJointDoF());

	controller_state_pub_->msg_.desired.contacts.resize(fbs_->getNumberOfEndEffectors());
	controller_state_pub_->msg_.actual.contacts.resize(fbs_->getNumberOfEndEffectors());
	controller_state_pub_->msg_.error.contacts.resize(fbs_->getNumberOfEndEffectors());

	controller_state_pub_->msg_.command.resize(fbs_->getJointDoF());
	controller_state_pub_->unlock();

	init_controller_state_pub_ = true;
}

void ControllerCommons::initWholeBodyStatePublisher(ros::NodeHandle node,
													const dwl::model::FloatingBaseSystem& fbs)
{
	// Getting the publish period
	if (!node.getParam("publish_rate", robot_publish_rate_)) {
		ROS_WARN("Parameter of WholeBodyState publisher 'publish_rate' not set"
				" (default value is 250). It has to be defined in"
				" %s/publish_rate", node.getNamespace().c_str());
	}

	// Setting the floating-base system info
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
	wb_iface_.reset(fbs);

	// Getting the robot namespace. Note that this node is deploy in robot_ns/controller_ns
	std::string robot_namespace = ros::names::parentNamespace(node.getNamespace());
	ros::NodeHandle robot_node(robot_namespace);

	// Initializing the real-time publisher
	robot_state_pub_.reset(new
			realtime_tools::RealtimePublisher<dwl_msgs::WholeBodyState> (robot_node, "robot_states", 1));
	robot_state_pub_->lock();

	robot_state_pub_->msg_.joints.resize(fbs_->getJointDoF());
	robot_state_pub_->msg_.contacts.resize(fbs_->getNumberOfEndEffectors());

	robot_state_pub_->unlock();

	init_robot_state_pub_ = true;
}


void ControllerCommons::initStateEstimationPublisher(ros::NodeHandle node)
{
	// Getting the publish period
	if (!node.getParam("publish_rate", odom_publish_rate_)) {
		ROS_WARN("Parameter of StateEstimation publisher 'publish_rate' not set"
				" (default value is 250). It has to be defined in"
				" %s/publish_rate", node.getNamespace().c_str());
	}

	// Initializing the real-time publisher
	ros::NodeHandle nh; //Nodehandle without prefix
	base_state_pub_.reset(new
			realtime_tools::RealtimePublisher<nav_msgs::Odometry> (node, "odom", 1));
    pose_covariance_pub_.reset(new
            realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped> (node, "pose", 1));
	tf_pub_.reset(new
			realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> (nh, "tf", 100));

	init_odom_state_pub_ = true;
}


void ControllerCommons::initImuPublisher(ros::NodeHandle node,
										 std::string imu_frame)
{
	imu_state_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu> (node, "imu", 1));
	imu_state_pub_->lock();
		imu_state_pub_->msg_.header.frame_id = imu_frame;
	imu_state_pub_->unlock();

	init_imu_state_pub_ = true;
}


void ControllerCommons::initStateEstimationSubscriber(ros::NodeHandle node)
{
	base_state_sub_ = node.subscribe<nav_msgs::Odometry> ("odom", 1,
			&ControllerCommons::setBaseStateCB, this, ros::TransportHints().tcpNoDelay());
}


void ControllerCommons::initMotionPlanSubscriber(ros::NodeHandle node,
												 const dwl::model::FloatingBaseSystem& fbs)
{
	plan_sub_ = node.subscribe<dwl_msgs::WholeBodyTrajectory> ("plan", 1,
			&ControllerCommons::setPlanCB, this, ros::TransportHints().tcpNoDelay());

	// Setting the floating-base system info
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
	wb_iface_.reset(fbs);
}


void ControllerCommons::publishControllerState(const ros::Time& time,
											   const dwl::WholeBodyState& current_state,
											   const dwl::WholeBodyState& desired_state,
											   const dwl::WholeBodyState& error_state,
											   const Eigen::VectorXd& feedforward_cmd,
											   const Eigen::VectorXd& feedback_cmd,
											   const Eigen::VectorXd& command_eff)
{
	// Sanity check of the publisher initialization
	if (!init_controller_state_pub_) {
		ROS_WARN("Could not published the controller state because it was not initialized");
		return;
	}

	// limit rate of publishing
	if (controller_publish_rate_ > 0.0 && last_controller_publish_time_ +
			ros::Duration(1.0 / controller_publish_rate_) < time) {
		// try to publish
		if (controller_state_pub_->trylock()) {
			// we're actually publishing, so increment time
			last_controller_publish_time_ += ros::Duration(1.0 / controller_publish_rate_);
			controller_state_pub_->msg_.header.stamp = time;

			// Converting the desired, actual and error whole-body statees
			wb_iface_.writeToMessage(controller_state_pub_->msg_.desired, desired_state);
			wb_iface_.writeToMessage(controller_state_pub_->msg_.actual, current_state);
			wb_iface_.writeToMessage(controller_state_pub_->msg_.error, error_state);

			// Populating joint command messages
			for (dwl::model::ElementId::const_iterator jnt_it = fbs_->getJoints().begin();
					jnt_it != fbs_->getJoints().end(); ++jnt_it) {
				std::string name = jnt_it->first;
				unsigned int id = jnt_it->second;

				controller_state_pub_->msg_.command[id].name = name;
				controller_state_pub_->msg_.command[id].total = command_eff(id);
				controller_state_pub_->msg_.command[id].feedforward = feedforward_cmd(id);
				controller_state_pub_->msg_.command[id].feedback = feedback_cmd(id);
			}

			controller_state_pub_->unlockAndPublish();
		}
	}
}


void ControllerCommons::publishWholeBodyState(const ros::Time& time,
											  const dwl::WholeBodyState& state)
{
	// Sanity check of the publisher initialization
	if (!init_robot_state_pub_) {
		ROS_WARN("Could not published the whole-body state because it was not initialized");
		return;
	}

	// limit rate of publishing
	if (robot_publish_rate_ > 0.0 && last_robot_publish_time_ +
			ros::Duration(1.0 / robot_publish_rate_) < time) {
		// try to publish
		if (robot_state_pub_->trylock()) {
			// we're actually publishing, so increment time
			last_robot_publish_time_ += ros::Duration(1.0 / robot_publish_rate_);
			robot_state_pub_->msg_.header.stamp = time;
			robot_state_pub_->msg_.header.frame_id = fbs_->getFloatingBaseName();

			wb_iface_.writeToMessage(robot_state_pub_->msg_, state);

			robot_state_pub_->unlockAndPublish();
		}
	}
}


void ControllerCommons::publishStateEstimation(const ros::Time& time,
											   const Eigen::Vector6d& base_pos,
											   const Eigen::Vector6d& base_vel)
{
	// Sanity check of the publisher initialization
	if (!init_odom_state_pub_) {
		ROS_WARN("Could not published the state estimation because it was not initialized");
		return;
	}

	// limit rate of publishing
	if (odom_publish_rate_ > 0.0 && last_odom_publish_time_ +
			ros::Duration(1.0 / odom_publish_rate_) < time) {
		// Computing the orientation in quaternion
		Eigen::Vector3d rpy;
		rpy << base_pos(dwl::rbd::AX_V), base_pos(dwl::rbd::AY_V), base_pos(dwl::rbd::AZ_V);
		Eigen::Quaterniond base_quat = dwl::math::getQuaternion(rpy);

		// try to publish
		if (base_state_pub_->trylock()) {
			// we're actually publishing, so increment time
			last_odom_publish_time_ += ros::Duration(1.0 / odom_publish_rate_);
			base_state_pub_->msg_.header.stamp = time;
			base_state_pub_->msg_.header.frame_id = "world";

			base_state_pub_->msg_.pose.pose.position.x = base_pos(dwl::rbd::LX_V);
			base_state_pub_->msg_.pose.pose.position.y = base_pos(dwl::rbd::LY_V);
			base_state_pub_->msg_.pose.pose.position.z = base_pos(dwl::rbd::LZ_V);
			base_state_pub_->msg_.pose.pose.orientation.x = base_quat.x();
			base_state_pub_->msg_.pose.pose.orientation.y = base_quat.y();
			base_state_pub_->msg_.pose.pose.orientation.z = base_quat.z();
			base_state_pub_->msg_.pose.pose.orientation.w = base_quat.w();
			base_state_pub_->msg_.twist.twist.linear.x = base_vel(dwl::rbd::LX_V);
			base_state_pub_->msg_.twist.twist.linear.y = base_vel(dwl::rbd::LY_V);
			base_state_pub_->msg_.twist.twist.linear.z = base_vel(dwl::rbd::LZ_V);
			base_state_pub_->msg_.twist.twist.angular.x = base_vel(dwl::rbd::AX_V);
			base_state_pub_->msg_.twist.twist.angular.y = base_vel(dwl::rbd::AY_V);
			base_state_pub_->msg_.twist.twist.angular.z = base_vel(dwl::rbd::AZ_V);
			base_state_pub_->unlockAndPublish();
		}

        // try to publish
        if (pose_covariance_pub_->trylock()) {
            // we're actually publishing, so increment time
            last_odom_publish_time_ += ros::Duration(1.0 / odom_publish_rate_);
            pose_covariance_pub_->msg_.header.stamp = time;
            pose_covariance_pub_->msg_.header.frame_id = "world";

            pose_covariance_pub_->msg_.pose.pose.position.x = base_pos(dwl::rbd::LX_V);
            pose_covariance_pub_->msg_.pose.pose.position.y = base_pos(dwl::rbd::LY_V);
            pose_covariance_pub_->msg_.pose.pose.position.z = base_pos(dwl::rbd::LZ_V);
            pose_covariance_pub_->msg_.pose.pose.orientation.x = base_quat.x();
            pose_covariance_pub_->msg_.pose.pose.orientation.y = base_quat.y();
            pose_covariance_pub_->msg_.pose.pose.orientation.z = base_quat.z();
            pose_covariance_pub_->msg_.pose.pose.orientation.w = base_quat.w();
            pose_covariance_pub_->unlockAndPublish();
        }

		// TF message
		geometry_msgs::TransformStamped tf_msg;
		tf_msg.header.stamp = time;
		tf_msg.child_frame_id = "base_link";
		tf_msg.header.frame_id = "world";

		tf_msg.transform.translation.x = base_pos(dwl::rbd::LX_V);
		tf_msg.transform.translation.y = base_pos(dwl::rbd::LY_V);
		tf_msg.transform.translation.z = base_pos(dwl::rbd::LZ_V);
		tf_msg.transform.rotation.x = base_quat.x();
		tf_msg.transform.rotation.y = base_quat.y();
		tf_msg.transform.rotation.z = base_quat.z();
		tf_msg.transform.rotation.w = base_quat.w();
		
		tf2_msgs::TFMessage tf2_msg;
		tf2_msg.transforms.push_back(tf_msg);
		if (tf_pub_->trylock()) {
			tf_pub_->msg_ = tf2_msg;
			tf_pub_->unlockAndPublish();
		}
	}
}


void ControllerCommons::publishImuState(const ros::Time& time,
										const struct ImuData& imu)
{
	// Sanity check of the publisher initialization
	if (!init_imu_state_pub_) {
		ROS_WARN("Could not published the imu state because it was not initialized");
		return;
	}

	// limit rate of publishing
	if (imu_publish_rate_ > 0.0 && last_imu_publish_time_ +
			ros::Duration(1.0 / imu_publish_rate_) < time) {
		// try to publish
		if (imu_state_pub_->trylock()) {
			// we're actually publishing, so increment time
			last_imu_publish_time_ += ros::Duration(1.0 / odom_publish_rate_);
			imu_state_pub_->msg_.header.stamp = time;
			imu_state_pub_->msg_.orientation.x = imu.orientation.x();
			imu_state_pub_->msg_.orientation.y = imu.orientation.y();
			imu_state_pub_->msg_.orientation.z = imu.orientation.z();
			imu_state_pub_->msg_.orientation.w = imu.orientation.w();
			imu_state_pub_->msg_.angular_velocity.x = imu.angular_velocity(dwl::rbd::X);
			imu_state_pub_->msg_.angular_velocity.y = imu.angular_velocity(dwl::rbd::Y);
			imu_state_pub_->msg_.angular_velocity.z = imu.angular_velocity(dwl::rbd::Z);
			imu_state_pub_->msg_.linear_acceleration.x = imu.linear_acceleration(dwl::rbd::X);
			imu_state_pub_->msg_.linear_acceleration.y = imu.linear_acceleration(dwl::rbd::Y);
			imu_state_pub_->msg_.linear_acceleration.z = imu.linear_acceleration(dwl::rbd::Z);
			imu_state_pub_->unlockAndPublish();
		}
	}
}


void ControllerCommons::updateStateEstimationSubscription(dwl::SE3& base_pos,
														  dwl::Motion& base_vel)
{
	// Putting the information inside Pose and Twist messages
	geometry_msgs::Pose pose;
	pose.position = base_state_.pose.pose.position;
	pose.orientation = base_state_.pose.pose.orientation;
	geometry_msgs::Twist twist;
	twist.linear = base_state_.twist.twist.linear;
	twist.angular = base_state_.twist.twist.angular;

	// Updating the base position and velocities
	wb_iface_.writeFromMessage(base_pos, pose);
	wb_iface_.writeFromMessage(base_vel, twist);
}


void ControllerCommons::updateMotionPlanSubscription(dwl::WholeBodyState& state)
{
	// Checks if there is a new plan message
	if (new_plan_) {
		// Setting the plan to be updated
		plan_ = *plan_buffer_.readFromRT();
		setPlan(plan_);
		new_plan_ = false;
	}

	// Updates the motion plan
	updatePlan(state);
}


void ControllerCommons::setPlan(const dwl_msgs::WholeBodyTrajectory& plan)
{
	// Getting the number of points in the trajectory
	num_traj_points_ = plan.trajectory.size();

	// This is a sanity check. If there is not trajectory information, we go out immediately
	if (num_traj_points_ == 0)
		return;

	// Setting the motion plan
	plan_ = plan;

	// Resetting the trajectory counter
	trajectory_counter_ = 0;
}


void ControllerCommons::updatePlan(dwl::WholeBodyState& state)
{
	// This is a sanity check. If there is not trajectory information, we go out immediately
	if (num_traj_points_ == 0)
		return;

	// Getting the actual desired whole-body state
	dwl_msgs::WholeBodyState msg = plan_.trajectory[trajectory_counter_];

	// Converting the WholeBodyState msg
	wb_iface_.writeFromMessage(state, msg);

	// The trajectory counter is set to zero once the trajectory is finished
	trajectory_counter_++;
	if (trajectory_counter_ >= num_traj_points_) {
		// Resetting values
		num_traj_points_ = 0;
		trajectory_counter_ = 0;
	}
}


void ControllerCommons::setBaseStateCB(const nav_msgs::OdometryConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	base_state_buffer_.writeFromNonRT(*msg);

	base_state_ = *base_state_buffer_.readFromNonRT();

	if (!init_base_state_)
		init_base_state_ = true;
}


void ControllerCommons::setPlanCB(const dwl_msgs::WholeBodyTrajectoryConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	plan_buffer_.writeFromNonRT(*msg);
	new_plan_ = true;
}

} //@namespace dwl_msgs
