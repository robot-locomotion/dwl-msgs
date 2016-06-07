#include <dwl_msgs/ControllerCommons.h>


namespace dwl_msgs
{

ControllerCommons::ControllerCommons() : new_plan_(false), init_base_state_(false),
		num_traj_points_(0), trajectory_counter_(0), controller_publish_rate_(50),
		robot_publish_rate_(250), odom_publish_rate_(250), imu_publish_rate_(250),
		init_controller_state_pub_(false), init_robot_state_pub_(false),
		init_odom_state_pub_(false), init_imu_state_pub_(false)
{

}


ControllerCommons::~ControllerCommons()
{

}


void ControllerCommons::initControllerStatePublisher(ros::NodeHandle node,
													 dwl::model::FloatingBaseSystem& system)
{
	// Getting the publish period
	if (!node.getParam("publish_rate", controller_publish_rate_)) {
		ROS_WARN("Parameter of ControllerState publisher 'publish_rate' not set"
				" (default value is 50). It has to be defined in"
				" %s/publish_rate", node.getNamespace().c_str());
	}

	// Setting the floating-base system info
	system_ = system;

	// Initializing the real-time publisher
	controller_state_pub_.reset(new
			realtime_tools::RealtimePublisher<dwl_msgs::WholeBodyController> (node, "state", 1));
	controller_state_pub_->lock();

	controller_state_pub_->msg_.desired.base.resize(6);
	controller_state_pub_->msg_.actual.base.resize(6);
	controller_state_pub_->msg_.error.base.resize(6);

	controller_state_pub_->msg_.desired.joints.resize(system_.getJointDoF());
	controller_state_pub_->msg_.actual.joints.resize(system_.getJointDoF());
	controller_state_pub_->msg_.error.joints.resize(system_.getJointDoF());

	controller_state_pub_->msg_.desired.contacts.resize(system_.getNumberOfEndEffectors());
	controller_state_pub_->msg_.actual.contacts.resize(system_.getNumberOfEndEffectors());
	controller_state_pub_->msg_.error.contacts.resize(system_.getNumberOfEndEffectors());

	controller_state_pub_->msg_.command.resize(system_.getJointDoF());
	controller_state_pub_->unlock();

	init_controller_state_pub_ = true;
}

void ControllerCommons::initWholeBodyStatePublisher(ros::NodeHandle node,
													dwl::model::FloatingBaseSystem& system)
{
	// Getting the publish period
	if (!node.getParam("publish_rate", robot_publish_rate_)) {
		ROS_WARN("Parameter of WholeBodyState publisher 'publish_rate' not set"
				" (default value is 50). It has to be defined in"
				" %s/publish_rate", node.getNamespace().c_str());
	}

	// Setting the floating-base system info
	system_ = system;

	// Getting the robot namespace. Note that this node is deploy in robot_ns/controller_ns
	std::string robot_namespace = ros::names::parentNamespace(node.getNamespace());
	ros::NodeHandle robot_node(robot_namespace);

	// Initializing the real-time publisher
	robot_state_pub_.reset(new
			realtime_tools::RealtimePublisher<dwl_msgs::WholeBodyState> (robot_node, "robot_states", 1));
	robot_state_pub_->lock();

	robot_state_pub_->msg_.base.resize(6);
	robot_state_pub_->msg_.joints.resize(system_.getJointDoF());
	robot_state_pub_->msg_.contacts.resize(system_.getNumberOfEndEffectors());

	robot_state_pub_->unlock();

	init_robot_state_pub_ = true;
}


void ControllerCommons::initStateEstimationPublisher(ros::NodeHandle node)
{
	// Getting the publish period
	if (!node.getParam("publish_rate", odom_publish_rate_)) {
		ROS_WARN("Parameter of StateEstimation publisher 'publish_rate' not set"
				" (default value is 50). It has to be defined in"
				" %s/publish_rate", node.getNamespace().c_str());
	}

	// Initializing the real-time publisher
	base_state_pub_.reset(new
			realtime_tools::RealtimePublisher<nav_msgs::Odometry> (node, "odom", 1));

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
												 dwl::model::FloatingBaseSystem& system)
{
	plan_sub_ = node.subscribe<dwl_msgs::WholeBodyTrajectory> ("plan", 1,
			&ControllerCommons::setPlanCB, this, ros::TransportHints().tcpNoDelay());

	// Setting the floating-base system info
	system_ = system;
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

			// Populating base states messages
			unsigned int counter = 0;
			for (unsigned int i = 0; i < 6; i++) {
				dwl::rbd::Coords6d base_id = dwl::rbd::Coords6d(i);
				dwl::model::FloatingBaseJoint base_joint = system_.getFloatingBaseJoint(base_id);

				if (base_joint.active) {
					controller_state_pub_->msg_.desired.base[counter].name = base_joint.name;
					controller_state_pub_->msg_.desired.base[counter].id = base_id;
					controller_state_pub_->msg_.desired.base[counter].position = desired_state.base_pos(i);
					controller_state_pub_->msg_.desired.base[counter].velocity = desired_state.base_vel(i);
					controller_state_pub_->msg_.desired.base[counter].acceleration = desired_state.base_acc(i);

					controller_state_pub_->msg_.actual.base[counter].id = base_id;
					controller_state_pub_->msg_.actual.base[counter].name = base_joint.name;
					controller_state_pub_->msg_.actual.base[counter].position = current_state.base_pos(i);
					controller_state_pub_->msg_.actual.base[counter].velocity = current_state.base_vel(i);
					controller_state_pub_->msg_.actual.base[counter].acceleration = current_state.base_acc(i);

					controller_state_pub_->msg_.error.base[counter].id = base_id;
					controller_state_pub_->msg_.error.base[counter].name = base_joint.name;
					controller_state_pub_->msg_.error.base[counter].position = error_state.base_pos(i);
					controller_state_pub_->msg_.error.base[counter].velocity = error_state.base_vel(i);

					++counter;
				}
			}

			// Populating joint states messages
			dwl::urdf_model::JointID joint_links = system_.getJoints();
			for (dwl::urdf_model::JointID::const_iterator joint_it = joint_links.begin();
					joint_it != joint_links.end(); joint_it++) {
				std::string name = joint_it->first;
				unsigned int id = joint_it->second;

				controller_state_pub_->msg_.desired.joints[id].name = name;
				controller_state_pub_->msg_.desired.joints[id].position = desired_state.joint_pos(id);
				controller_state_pub_->msg_.desired.joints[id].velocity = desired_state.joint_vel(id);
				controller_state_pub_->msg_.desired.joints[id].acceleration = desired_state.joint_acc(id);
				controller_state_pub_->msg_.desired.joints[id].effort = desired_state.joint_eff(id);

				controller_state_pub_->msg_.actual.joints[id].name = name;
				controller_state_pub_->msg_.actual.joints[id].position = current_state.joint_pos(id);
				controller_state_pub_->msg_.actual.joints[id].velocity = current_state.joint_vel(id);
				controller_state_pub_->msg_.actual.joints[id].acceleration = current_state.joint_acc(id);
				controller_state_pub_->msg_.actual.joints[id].effort = current_state.joint_eff(id);

				controller_state_pub_->msg_.error.joints[id].name = name;
				controller_state_pub_->msg_.error.joints[id].position = error_state.joint_pos(id);
				controller_state_pub_->msg_.error.joints[id].velocity = error_state.joint_vel(id);

				controller_state_pub_->msg_.command[id].name = name;
				controller_state_pub_->msg_.command[id].total = command_eff(id);
				controller_state_pub_->msg_.command[id].feedforward = feedforward_cmd(id);
				controller_state_pub_->msg_.command[id].feedback = feedback_cmd(id);
			}

			// Populating contact states messages
			dwl::urdf_model::LinkID contact_links = system_.getEndEffectors();
			for (dwl::urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
					contact_it != contact_links.end(); contact_it++) {
				std::string name = contact_it->first;
				unsigned int id = contact_it->second;

				// Filling the name of the contact
				controller_state_pub_->msg_.desired.contacts[id].name = name;
				controller_state_pub_->msg_.actual.contacts[id].name = name;
				controller_state_pub_->msg_.error.contacts[id].name = name;

				// Defining the contact state vectors
				Eigen::Vector3d position, velocity, acceleration;
				dwl::rbd::Vector6d effort;

				// Defining the contact iterators
				dwl::rbd::BodyVector::const_iterator pos_it, vel_it, acc_it;
				dwl::rbd::BodyWrench::const_iterator eff_it;

				// Filling the desired contact state
				pos_it = desired_state.contact_pos.find(name);
				vel_it = desired_state.contact_vel.find(name);
				acc_it = desired_state.contact_acc.find(name);
				eff_it = desired_state.contact_eff.find(name);
				if (pos_it != desired_state.contact_pos.end()) {
					position = pos_it->second.tail(3);
					controller_state_pub_->msg_.desired.contacts[id].position.x = position(dwl::rbd::X);
					controller_state_pub_->msg_.desired.contacts[id].position.y = position(dwl::rbd::Y);
					controller_state_pub_->msg_.desired.contacts[id].position.z = position(dwl::rbd::Z);
				}
				if (vel_it != desired_state.contact_vel.end()) {
					velocity = vel_it->second.tail(3);
					controller_state_pub_->msg_.desired.contacts[id].velocity.x = velocity(dwl::rbd::X);
					controller_state_pub_->msg_.desired.contacts[id].velocity.y = velocity(dwl::rbd::Y);
					controller_state_pub_->msg_.desired.contacts[id].velocity.z = velocity(dwl::rbd::Z);
				}
				if (acc_it != desired_state.contact_acc.end()) {
					acceleration = acc_it->second.tail(3);
					controller_state_pub_->msg_.desired.contacts[id].acceleration.x = acceleration(dwl::rbd::X);
					controller_state_pub_->msg_.desired.contacts[id].acceleration.y = acceleration(dwl::rbd::Y);
					controller_state_pub_->msg_.desired.contacts[id].acceleration.z = acceleration(dwl::rbd::Z);
				}
				if (eff_it != desired_state.contact_eff.end()) {
					effort = eff_it->second;
					controller_state_pub_->msg_.desired.contacts[id].wrench.force.x = effort(dwl::rbd::LX);
					controller_state_pub_->msg_.desired.contacts[id].wrench.force.y = effort(dwl::rbd::LY);
					controller_state_pub_->msg_.desired.contacts[id].wrench.force.z = effort(dwl::rbd::LZ);
					controller_state_pub_->msg_.desired.contacts[id].wrench.torque.x = effort(dwl::rbd::AX);
					controller_state_pub_->msg_.desired.contacts[id].wrench.torque.y = effort(dwl::rbd::AY);
					controller_state_pub_->msg_.desired.contacts[id].wrench.torque.z = effort(dwl::rbd::AZ);
				}

				// Filling the current contact state
				pos_it = current_state.contact_pos.find(name);
				vel_it = current_state.contact_vel.find(name);
				acc_it = current_state.contact_acc.find(name);
				eff_it = current_state.contact_eff.find(name);
				if (pos_it != current_state.contact_pos.end()) {
					position = pos_it->second.tail(3);
					controller_state_pub_->msg_.actual.contacts[id].position.x = position(dwl::rbd::X);
					controller_state_pub_->msg_.actual.contacts[id].position.y = position(dwl::rbd::Y);
					controller_state_pub_->msg_.actual.contacts[id].position.z = position(dwl::rbd::Z);
				}
				if (vel_it != current_state.contact_vel.end()) {
					velocity = vel_it->second.tail(3);
					controller_state_pub_->msg_.actual.contacts[id].velocity.x = velocity(dwl::rbd::X);
					controller_state_pub_->msg_.actual.contacts[id].velocity.y = velocity(dwl::rbd::Y);
					controller_state_pub_->msg_.actual.contacts[id].velocity.z = velocity(dwl::rbd::Z);
				}
				if (acc_it != current_state.contact_acc.end()) {
					acceleration = acc_it->second.tail(3);
					controller_state_pub_->msg_.actual.contacts[id].acceleration.x = acceleration(dwl::rbd::X);
					controller_state_pub_->msg_.actual.contacts[id].acceleration.y = acceleration(dwl::rbd::Y);
					controller_state_pub_->msg_.actual.contacts[id].acceleration.z = acceleration(dwl::rbd::Z);
				}
				if (eff_it != current_state.contact_eff.end()) {
					effort = eff_it->second;
					controller_state_pub_->msg_.actual.contacts[id].wrench.force.x = effort(dwl::rbd::LX);
					controller_state_pub_->msg_.actual.contacts[id].wrench.force.y = effort(dwl::rbd::LY);
					controller_state_pub_->msg_.actual.contacts[id].wrench.force.z = effort(dwl::rbd::LZ);
					controller_state_pub_->msg_.actual.contacts[id].wrench.torque.x = effort(dwl::rbd::AX);
					controller_state_pub_->msg_.actual.contacts[id].wrench.torque.y = effort(dwl::rbd::AY);
					controller_state_pub_->msg_.actual.contacts[id].wrench.torque.z = effort(dwl::rbd::AZ);
				}

				// Filling the error contact state
				pos_it = error_state.contact_pos.find(name);
				vel_it = error_state.contact_vel.find(name);
				acc_it = error_state.contact_acc.find(name);
				eff_it = error_state.contact_eff.find(name);
				if (pos_it != error_state.contact_pos.end()) {
					position = pos_it->second.tail(3);
					controller_state_pub_->msg_.error.contacts[id].position.x = position(dwl::rbd::X);
					controller_state_pub_->msg_.error.contacts[id].position.y = position(dwl::rbd::Y);
					controller_state_pub_->msg_.error.contacts[id].position.z = position(dwl::rbd::Z);
				}
				if (vel_it != error_state.contact_vel.end()) {
					velocity = vel_it->second.tail(3);
					controller_state_pub_->msg_.error.contacts[id].velocity.x = velocity(dwl::rbd::X);
					controller_state_pub_->msg_.error.contacts[id].velocity.y = velocity(dwl::rbd::Y);
					controller_state_pub_->msg_.error.contacts[id].velocity.z = velocity(dwl::rbd::Z);
				}
				if (acc_it != error_state.contact_acc.end()) {
					acceleration = acc_it->second.tail(3);
					controller_state_pub_->msg_.error.contacts[id].acceleration.x = acceleration(dwl::rbd::X);
					controller_state_pub_->msg_.error.contacts[id].acceleration.y = acceleration(dwl::rbd::Y);
					controller_state_pub_->msg_.error.contacts[id].acceleration.z = acceleration(dwl::rbd::Z);
				}
				if (eff_it != error_state.contact_eff.end()) {
					effort = eff_it->second;
					controller_state_pub_->msg_.error.contacts[id].wrench.force.x = effort(dwl::rbd::LX);
					controller_state_pub_->msg_.error.contacts[id].wrench.force.y = effort(dwl::rbd::LY);
					controller_state_pub_->msg_.error.contacts[id].wrench.force.z = effort(dwl::rbd::LZ);
					controller_state_pub_->msg_.error.contacts[id].wrench.torque.x = effort(dwl::rbd::AX);
					controller_state_pub_->msg_.error.contacts[id].wrench.torque.y = effort(dwl::rbd::AY);
					controller_state_pub_->msg_.error.contacts[id].wrench.torque.z = effort(dwl::rbd::AZ);
				}
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
			robot_state_pub_->msg_.header.stamp = ros::Time::now();
			robot_state_pub_->msg_.header.frame_id = system_.getFloatingBaseBody();

			// we're actually publishing, so increment time
			last_robot_publish_time_ += ros::Duration(1.0 / robot_publish_rate_);
			robot_state_pub_->msg_.time = state.time;

			// Populating base states messages
			unsigned int counter = 0;
			for (unsigned int i = 0; i < 6; i++) {
				dwl::rbd::Coords6d base_id = dwl::rbd::Coords6d(i);
				dwl::model::FloatingBaseJoint base_joint = system_.getFloatingBaseJoint(base_id);

				if (base_joint.active) {
					robot_state_pub_->msg_.base[counter].name = base_joint.name;
					robot_state_pub_->msg_.base[counter].id = base_id;
					robot_state_pub_->msg_.base[counter].position = state.base_pos(i);
					robot_state_pub_->msg_.base[counter].velocity = state.base_vel(i);
					robot_state_pub_->msg_.base[counter].acceleration = state.base_acc(i);

					++counter;
				}
			}

			// Populating joint states messages
			dwl::urdf_model::JointID joint_links = system_.getJoints();
			if (state.getJointDof() == system_.getJointDoF()) { //sanity check
				for (dwl::urdf_model::JointID::const_iterator joint_it = joint_links.begin();
						joint_it != joint_links.end(); joint_it++) {
					std::string name = joint_it->first;
					unsigned int id = joint_it->second;

					robot_state_pub_->msg_.joints[id].name = name;
					robot_state_pub_->msg_.joints[id].position = state.joint_pos(id);
					robot_state_pub_->msg_.joints[id].velocity = state.joint_vel(id);
					robot_state_pub_->msg_.joints[id].acceleration = state.joint_acc(id);
					robot_state_pub_->msg_.joints[id].effort = state.joint_eff(id);
				}
			} else {
				ROS_WARN("Could not published the joint state in the WholeBodyState message because"
						" a wrong dimension");
			}

			// Populating contact states messages
			dwl::urdf_model::LinkID contact_links = system_.getEndEffectors();
			for (dwl::urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
					contact_it != contact_links.end(); contact_it++) {
				std::string name = contact_it->first;
				unsigned int id = contact_it->second;

				// Filling the name of the contact
				robot_state_pub_->msg_.contacts[id].name = name;

				// Defining the contact state vectors
				Eigen::Vector3d position, velocity, acceleration;
				dwl::rbd::Vector6d effort;

				// Defining the contact iterators
				dwl::rbd::BodyVector::const_iterator pos_it, vel_it, acc_it;
				dwl::rbd::BodyWrench::const_iterator eff_it;

				// Filling the contact state
				pos_it = state.contact_pos.find(name);
				vel_it = state.contact_vel.find(name);
				acc_it = state.contact_acc.find(name);
				eff_it = state.contact_eff.find(name);
				if (pos_it != state.contact_pos.end()) {
					position = pos_it->second.tail(3);
					robot_state_pub_->msg_.contacts[id].position.x = position(dwl::rbd::X);
					robot_state_pub_->msg_.contacts[id].position.y = position(dwl::rbd::Y);
					robot_state_pub_->msg_.contacts[id].position.z = position(dwl::rbd::Z);
				}
				if (vel_it != state.contact_vel.end()) {
					velocity = vel_it->second.tail(3);
					robot_state_pub_->msg_.contacts[id].velocity.x = velocity(dwl::rbd::X);
					robot_state_pub_->msg_.contacts[id].velocity.y = velocity(dwl::rbd::Y);
					robot_state_pub_->msg_.contacts[id].velocity.z = velocity(dwl::rbd::Z);
				}
				if (acc_it != state.contact_acc.end()) {
					acceleration = acc_it->second.tail(3);
					robot_state_pub_->msg_.contacts[id].acceleration.x = acceleration(dwl::rbd::X);
					robot_state_pub_->msg_.contacts[id].acceleration.y = acceleration(dwl::rbd::Y);
					robot_state_pub_->msg_.contacts[id].acceleration.z = acceleration(dwl::rbd::Z);
				}
				if (eff_it != state.contact_eff.end()) {
					effort = eff_it->second;
					robot_state_pub_->msg_.contacts[id].wrench.force.x = effort(dwl::rbd::LX);
					robot_state_pub_->msg_.contacts[id].wrench.force.y = effort(dwl::rbd::LY);
					robot_state_pub_->msg_.contacts[id].wrench.force.z = effort(dwl::rbd::LZ);
					robot_state_pub_->msg_.contacts[id].wrench.torque.x = effort(dwl::rbd::AX);
					robot_state_pub_->msg_.contacts[id].wrench.torque.y = effort(dwl::rbd::AY);
					robot_state_pub_->msg_.contacts[id].wrench.torque.z = effort(dwl::rbd::AZ);
				}
			}

			robot_state_pub_->unlockAndPublish();
		}
	}
}


void ControllerCommons::publishStateEstimation(const ros::Time& time,
											   const dwl::rbd::Vector6d& base_pos,
											   const dwl::rbd::Vector6d& base_vel)
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
		rpy << base_pos(dwl::rbd::AX), base_pos(dwl::rbd::AY), base_pos(dwl::rbd::AZ);
		Eigen::Quaterniond base_quat = dwl::math::getQuaternion(rpy);

		// try to publish
		if (base_state_pub_->trylock()) {
			// we're actually publishing, so increment time
			last_controller_publish_time_ += ros::Duration(1.0 / odom_publish_rate_);
			base_state_pub_->msg_.header.stamp = time;

			base_state_pub_->msg_.header.stamp = time;
			base_state_pub_->msg_.header.frame_id = "world";
			base_state_pub_->msg_.pose.pose.position.x = base_pos(dwl::rbd::LX);
			base_state_pub_->msg_.pose.pose.position.y = base_pos(dwl::rbd::LY);
			base_state_pub_->msg_.pose.pose.position.z = base_pos(dwl::rbd::LZ);
			base_state_pub_->msg_.pose.pose.orientation.x = base_quat.x();
			base_state_pub_->msg_.pose.pose.orientation.y = base_quat.y();
			base_state_pub_->msg_.pose.pose.orientation.z = base_quat.z();
			base_state_pub_->msg_.pose.pose.orientation.w = base_quat.w();
			base_state_pub_->msg_.twist.twist.linear.x = base_vel(dwl::rbd::LX);
			base_state_pub_->msg_.twist.twist.linear.y = base_vel(dwl::rbd::LY);
			base_state_pub_->msg_.twist.twist.linear.z = base_vel(dwl::rbd::LZ);
			base_state_pub_->msg_.twist.twist.angular.x = base_vel(dwl::rbd::AX);
			base_state_pub_->msg_.twist.twist.angular.y = base_vel(dwl::rbd::AY);
			base_state_pub_->msg_.twist.twist.angular.z = base_vel(dwl::rbd::AZ);
			base_state_pub_->unlockAndPublish();
		}

		// TF message
		geometry_msgs::TransformStamped tf_msg;
		tf_msg.header.stamp = time;
		tf_msg.child_frame_id = "base_link";
		tf_msg.header.frame_id = "world";
		tf_msg.transform.translation.x = base_pos(dwl::rbd::LX);
		tf_msg.transform.translation.y = base_pos(dwl::rbd::LY);
		tf_msg.transform.translation.z = base_pos(dwl::rbd::LZ);
		tf_msg.transform.rotation.x = base_quat.x();
		tf_msg.transform.rotation.y = base_quat.y();
		tf_msg.transform.rotation.z = base_quat.z();
		tf_msg.transform.rotation.w = base_quat.w();
		odom_tf_broadcaster_.sendTransform(tf_msg);
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
			last_controller_publish_time_ += ros::Duration(1.0 / odom_publish_rate_);
			base_state_pub_->msg_.header.stamp = time;
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


void ControllerCommons::updateStateEstimationSubscription(dwl::rbd::Vector6d& base_pos,
											   	   	   	  dwl::rbd::Vector6d& base_vel)
{
	// Updating the base positions
	geometry_msgs::Quaternion q = base_state_.pose.pose.orientation;
	Eigen::Quaterniond base_quat(q.w, q.x, q.y, q.z);
	Eigen::Vector3d base_rpy = dwl::math::getRPY(base_quat);

	// The Roll Pitch Yaw convention defines a rotation about x0 (Roll) + a rotation about
	// y0 (Pitch) + a rotation about z0 (Yaw). So, RPY XYZ (gamma,beta,alpha) is equal to
	// Euler ZYX (alpha, beta, gamma)
	base_pos(dwl::rbd::AX) = base_rpy(0);
	base_pos(dwl::rbd::AY) = base_rpy(1);
	base_pos(dwl::rbd::AZ) = base_rpy(2);
	base_pos(dwl::rbd::LX) = base_state_.pose.pose.position.x;
	base_pos(dwl::rbd::LY) = base_state_.pose.pose.position.y;
	base_pos(dwl::rbd::LZ) = base_state_.pose.pose.position.z;

	// Updating the base velocities
	base_vel(dwl::rbd::AX) = base_state_.twist.twist.angular.x;
	base_vel(dwl::rbd::AY) = base_state_.twist.twist.angular.y;
	base_vel(dwl::rbd::AZ) = base_state_.twist.twist.angular.z;
	base_vel(dwl::rbd::LX) = base_state_.twist.twist.linear.x;
	base_vel(dwl::rbd::LY) = base_state_.twist.twist.linear.y;
	base_vel(dwl::rbd::LZ) = base_state_.twist.twist.linear.z;
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
	dwl_msgs::WholeBodyState state_msgs = plan_.trajectory[trajectory_counter_];

	// Updating the time information
	state.time = state_msgs.time;

	// Updating the base states
	unsigned int num_base = state_msgs.base.size();
	for (unsigned int base_idx = 0; base_idx < num_base; base_idx++) {
		// Getting the base message
		dwl_msgs::BaseState base_msgs = state_msgs.base[base_idx];

		unsigned int base_coord = base_msgs.id;
		state.base_pos(base_coord) = base_msgs.position;
		state.base_vel(base_coord) = base_msgs.velocity;
		state.base_acc(base_coord) = base_msgs.acceleration;
	}

	// Updating the joint states
	unsigned int num_joints = state_msgs.joints.size();
	if (num_joints == system_.getJointDoF()) { //safety check
		if (state.getJointDof() != num_joints) //safety check
			state.setJointDoF(num_joints);

		for (unsigned int joint_idx = 0; joint_idx < num_joints; joint_idx++) {
			// Getting the joint message
			dwl_msgs::JointState joint_msg = state_msgs.joints[joint_idx];
			unsigned int id = system_.getJoints().find(joint_msg.name)->second;

			state.joint_pos(id) = joint_msg.position;
			state.joint_vel(id) = joint_msg.velocity;
			state.joint_acc(id) = joint_msg.acceleration;
			state.joint_eff(id) = joint_msg.effort;
		}
	}

	// Updating the contact states
	Eigen::Vector3d contact_state;
	unsigned int num_contacts = state_msgs.contacts.size();
	for (unsigned int contact_idx = 0; contact_idx < num_contacts; contact_idx++) {
		// Getting the contact message
		dwl_msgs::ContactState contact_msg = state_msgs.contacts[contact_idx];

		std::string name = contact_msg.name;

		// Updating the contact position
		contact_state(dwl::rbd::X) = contact_msg.position.x;
		contact_state(dwl::rbd::Y) = contact_msg.position.y;
		contact_state(dwl::rbd::Z) = contact_msg.position.z;
		state.contact_pos[name] = contact_state;

		// Updating the contact velocity
		contact_state(dwl::rbd::X) = contact_msg.velocity.x;
		contact_state(dwl::rbd::Y) = contact_msg.velocity.y;
		contact_state(dwl::rbd::Z) = contact_msg.velocity.z;
		state.contact_vel[name] = contact_state;

		// Updating the contact acceleration
		contact_state(dwl::rbd::X) = contact_msg.acceleration.x;
		contact_state(dwl::rbd::Y) = contact_msg.acceleration.y;
		contact_state(dwl::rbd::Z) = contact_msg.acceleration.z;
		state.contact_acc[name] = contact_state;

		contact_state(dwl::rbd::X) = contact_msg.wrench.force.x;
		contact_state(dwl::rbd::Y) = contact_msg.wrench.force.y;
		contact_state(dwl::rbd::Z) = contact_msg.wrench.force.z;
		state.contact_eff[name] << 0, 0, 0, contact_state;
	}

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
