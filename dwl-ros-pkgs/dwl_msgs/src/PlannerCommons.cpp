#include <dwl_msgs/PlannerCommons.h>


namespace dwl_msgs
{

PlannerCommons::PlannerCommons() : init_motion_plan_pub_(false),
		init_reduced_plan_pub_(false), world_frame_id_("world"),
		received_robot_state_(false), received_controller_state_(false)
{

}


PlannerCommons::~PlannerCommons()
{

}


void PlannerCommons::initMotionPlanStatePublisher(ros::NodeHandle node_pub,
												  ros::NodeHandle node_params,
												  dwl::model::FloatingBaseSystem& system)
{
	// Setting the floating-base system info
	system_ = system;
	wb_iface_.reset(system_);

	// Initializing the publisher
	motion_plan_pub_ =
			node_pub.advertise<dwl_msgs::WholeBodyTrajectory>(node_pub.getNamespace() + "/plan", 1);

	// Reading the world frame id
	if (!node_params.getParam("world_frame", world_frame_id_))
		ROS_WARN("Setting up the default world frame (world)");

	init_motion_plan_pub_ = true;
}


void PlannerCommons::initReducedPlanPublisher(ros::NodeHandle node_pub,
											  ros::NodeHandle node_params)
{
	// Initializing the publisher
	reduced_plan_pub_ =
			node_pub.advertise<dwl_msgs::ReducedBodyTrajectory>(node_pub.getNamespace() + "/reduced_plan", 1);

	// Reading the world frame id
	if (!node_params.getParam("world_frame", world_frame_id_))
		ROS_WARN("Setting up the default world frame (world)");

	init_reduced_plan_pub_ = true;
}


void PlannerCommons::initRobotStateSubscriber(ros::NodeHandle node,
											  dwl::model::FloatingBaseSystem& system)
{
	// Setting the floating-base system info
	system_ = system;
	wb_iface_.reset(system_);

	robot_state_sub_ = node.subscribe<dwl_msgs::WholeBodyState>(node.getNamespace() + "/robot_states", 1,
			&PlannerCommons::setRobotStateCB, this, ros::TransportHints().tcpNoDelay());
}


void PlannerCommons::initControllertStateSubscriber(ros::NodeHandle node,
													dwl::model::FloatingBaseSystem& system)
{
	// Setting the floating-base system info
	system_ = system;
	wb_iface_.reset(system_);

	controller_state_sub_ = node.subscribe<dwl_msgs::WholeBodyController>(node.getNamespace() + "/state", 1,
			&PlannerCommons::setControllerStateCB, this, ros::TransportHints().tcpNoDelay());
}


void PlannerCommons::publishMotionPlan(const dwl::WholeBodyState& current_state,
									   const dwl::WholeBodyTrajectory& trajectory)
{
	// Sanity check of the publisher initialization
	if (!init_motion_plan_pub_) {
		ROS_WARN("Could not published the motion plan because it was not initialized");
		return;
	}

	// Waiting that the subscriber is connected for few msecs
	ros::Rate poll_rate(100);
	int counter = 0;
	while (motion_plan_pub_.getNumSubscribers() == 0 && counter <= 20) {
	    poll_rate.sleep();
	    counter++;
	}

	// Publishing the motion plan if there is at least one subscriber
	if (motion_plan_pub_.getNumSubscribers() > 0) {
		motion_plan_msg_.header.stamp = ros::Time::now();
		motion_plan_msg_.header.frame_id = world_frame_id_;

		// Filling the current state
		wb_iface_.writeToMessage(motion_plan_msg_.actual, current_state);

		// Filling the trajectory
		wb_iface_.writeToMessage(motion_plan_msg_, trajectory);

		// Publishing the motion plan
		motion_plan_pub_.publish(motion_plan_msg_);
	}
}


void PlannerCommons::publishReducedPlan(const dwl::ReducedBodyState& current_state,
										const dwl::ReducedBodyTrajectory& trajectory)
{
	// Sanity check of the publisher initialization
	if (!init_reduced_plan_pub_) {
		ROS_WARN("Could not published the stability trajectory because it was not initialized");
		return;
	}

	// Waiting that the subscriber is connected for few msecs
	ros::Rate poll_rate(100);
	int counter = 0;
	while (motion_plan_pub_.getNumSubscribers() == 0 && counter <= 20) {
	    poll_rate.sleep();
	    counter++;
	}

	// Publishing the reduced motion plan if there is at least one subscriber
	if (reduced_plan_pub_.getNumSubscribers() > 0) {
		reduced_plan_msg_.header.stamp = ros::Time::now();
		reduced_plan_msg_.header.frame_id = world_frame_id_;

		// Filling the current state
		writeReducedBodyStateMessage(reduced_plan_msg_.actual,
									 current_state);

		// Filling the trajectory
		reduced_plan_msg_.trajectory.resize(trajectory.size());
		for (unsigned int k = 0; k < trajectory.size(); k++)
			writeReducedBodyStateMessage(reduced_plan_msg_.trajectory[k],
										 trajectory[k]);

		// Publishing the motion plan
		reduced_plan_pub_.publish(reduced_plan_msg_);
	}
}


void PlannerCommons::writeReducedBodyStateMessage(dwl_msgs::ReducedBodyState& msg,
												  const dwl::ReducedBodyState& state)
{
	// Filling the time
	msg.time = state.time;

	// Filling the CoM position
	msg.center_of_mass.x = state.com_pos(dwl::rbd::X);
	msg.center_of_mass.y = state.com_pos(dwl::rbd::Y);
	msg.center_of_mass.z = state.com_pos(dwl::rbd::Z);

	// Filling the CoP position
	msg.center_of_pressure.x = state.cop(dwl::rbd::X);
	msg.center_of_pressure.y = state.cop(dwl::rbd::Y);
	msg.center_of_pressure.z = state.cop(dwl::rbd::Z);

	// Filling the support position
	msg.support_region.resize(state.support_region.size());
	unsigned int idx = 0;
	for (dwl::rbd::BodyVector3d::const_iterator vertex_it = state.support_region.begin();
			vertex_it != state.support_region.end(); vertex_it++) {
		Eigen::Vector3d vertex = vertex_it->second;
		msg.support_region[idx].x = vertex(dwl::rbd::X);
		msg.support_region[idx].y = vertex(dwl::rbd::Y);
		msg.support_region[idx].z = vertex(dwl::rbd::Z);
		++idx;
	}
}


void PlannerCommons::updateRobotStateSubscription(dwl::WholeBodyState& robot_state)
{
	// Setting the base states
	unsigned num_base = robot_state_msg_.base.size();
	for (unsigned int i = 0; i < num_base; i++) {
		unsigned base_id = robot_state_msg_.base[i].id;

		robot_state.base_pos(base_id) = robot_state_msg_.base[i].position;
		robot_state.base_vel(base_id) = robot_state_msg_.base[i].velocity;
		robot_state.base_acc(base_id) = robot_state_msg_.base[i].acceleration;
	}

	// Sanity check: checking the size of the joint states
	if (robot_state.joint_pos.size() != system_.getJointDoF())
		robot_state.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (robot_state.joint_vel.size() != system_.getJointDoF())
		robot_state.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (robot_state.joint_acc.size() != system_.getJointDoF())
		robot_state.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (robot_state.joint_eff.size() != system_.getJointDoF())
		robot_state.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());

	// Setting the joint states
	dwl::urdf_model::JointID joints = system_.getJoints();
	unsigned num_joints = robot_state_msg_.joints.size();
	robot_state.setJointDoF(num_joints);
	for (unsigned int i = 0; i < num_joints; i++) {
		std::string name = robot_state_msg_.joints[i].name;

		dwl::urdf_model::JointID::iterator joint_it = joints.find(name);
		unsigned int joint_id = joint_it->second;

		robot_state.joint_pos(joint_id) = robot_state_msg_.joints[i].position;
		robot_state.joint_vel(joint_id) = robot_state_msg_.joints[i].velocity;
		robot_state.joint_acc(joint_id) = robot_state_msg_.joints[i].acceleration;
		robot_state.joint_eff(joint_id) = robot_state_msg_.joints[i].effort;
	}

	// Setting the contact states
	unsigned int num_contacts = robot_state_msg_.contacts.size();
	for (unsigned int i = 0; i < num_contacts; i++) {
		// Getting the contact message
		dwl_msgs::ContactState contact_msg = robot_state_msg_.contacts[i];

		std::string name = contact_msg.name;
		dwl::urdf_model::LinkID contact_links = system_.getEndEffectors();

		// Updating the contact position
		Eigen::VectorXd position(3);
		position << contact_msg.position.x, contact_msg.position.y, contact_msg.position.z;
		robot_state.contact_pos[name] = position;

		// Updating the contact velocity
		Eigen::VectorXd velocity(3);
		velocity << contact_msg.velocity.x, contact_msg.velocity.y, contact_msg.velocity.z;
		robot_state.contact_vel[name] = velocity;

		// Updating the contact acceleration
		Eigen::VectorXd acceleration(3);
		acceleration << contact_msg.acceleration.x, contact_msg.acceleration.y, contact_msg.acceleration.z;
		robot_state.contact_acc[name] = acceleration;

		// Updating the contact wrench
		dwl::rbd::Vector6d effort;
		effort(dwl::rbd::LX) = contact_msg.wrench.force.x;
		effort(dwl::rbd::LY) = contact_msg.wrench.force.y;
		effort(dwl::rbd::LZ) = contact_msg.wrench.force.z;
		effort(dwl::rbd::AX) = contact_msg.wrench.torque.x;
		effort(dwl::rbd::AY) = contact_msg.wrench.torque.y;
		effort(dwl::rbd::AZ) = contact_msg.wrench.torque.z;
		robot_state.contact_eff[name] = effort;
	}
}


void PlannerCommons::updateControllerStateSubscription(dwl::WholeBodyState& desired,
													   dwl::WholeBodyState& actual,
													   dwl::WholeBodyState& error)
{
	// Setting the base states
	unsigned num_base = controller_state_msg_.actual.base.size();
	for (unsigned int i = 0; i < num_base; i++) {
		unsigned base_id = controller_state_msg_.actual.base[i].id;

		// Desired base state
		desired.base_pos(base_id) = controller_state_msg_.desired.base[i].position;
		desired.base_vel(base_id) = controller_state_msg_.desired.base[i].velocity;
		desired.base_acc(base_id) = controller_state_msg_.desired.base[i].acceleration;

		// Actual base state
		actual.base_pos(base_id) = controller_state_msg_.actual.base[i].position;
		actual.base_vel(base_id) = controller_state_msg_.actual.base[i].velocity;
		actual.base_acc(base_id) = controller_state_msg_.actual.base[i].acceleration;

		// Error base state
		error.base_pos(base_id) = controller_state_msg_.error.base[i].position;
		error.base_vel(base_id) = controller_state_msg_.error.base[i].velocity;
		error.base_acc(base_id) = controller_state_msg_.error.base[i].acceleration;
	}

	// Sanity check: checking the size of the joint states
	if (desired.joint_pos.size() != system_.getJointDoF())
		desired.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (desired.joint_vel.size() != system_.getJointDoF())
		desired.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (desired.joint_acc.size() != system_.getJointDoF())
		desired.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (desired.joint_eff.size() != system_.getJointDoF())
		desired.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (actual.joint_pos.size() != system_.getJointDoF())
		actual.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (actual.joint_vel.size() != system_.getJointDoF())
		actual.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (actual.joint_acc.size() != system_.getJointDoF())
		actual.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (actual.joint_eff.size() != system_.getJointDoF())
		actual.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (error.joint_pos.size() != system_.getJointDoF())
		error.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (error.joint_vel.size() != system_.getJointDoF())
		error.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (error.joint_acc.size() != system_.getJointDoF())
		error.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (error.joint_eff.size() != system_.getJointDoF())
		error.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());


	// Setting the joint states
	dwl::urdf_model::JointID joints = system_.getJoints();
	unsigned num_joints = controller_state_msg_.actual.joints.size();
	for (unsigned int i = 0; i < num_joints; i++) {
		std::string name = controller_state_msg_.actual.joints[i].name;

		dwl::urdf_model::JointID::iterator joint_it = joints.find(name);
		unsigned int joint_id = joint_it->second;

		// Desired joint states
		desired.joint_pos(joint_id) = controller_state_msg_.desired.joints[i].position;
		desired.joint_vel(joint_id) = controller_state_msg_.desired.joints[i].velocity;
		desired.joint_acc(joint_id) = controller_state_msg_.desired.joints[i].acceleration;
		desired.joint_eff(joint_id) = controller_state_msg_.desired.joints[i].effort;

		// Actual joint states
		actual.joint_pos(joint_id) = controller_state_msg_.actual.joints[i].position;
		actual.joint_vel(joint_id) = controller_state_msg_.actual.joints[i].velocity;
		actual.joint_acc(joint_id) = controller_state_msg_.actual.joints[i].acceleration;
		actual.joint_eff(joint_id) = controller_state_msg_.actual.joints[i].effort;

		// Error joint states
		error.joint_pos(joint_id) = controller_state_msg_.error.joints[i].position;
		error.joint_vel(joint_id) = controller_state_msg_.error.joints[i].velocity;
		error.joint_acc(joint_id) = controller_state_msg_.error.joints[i].acceleration;
		error.joint_eff(joint_id) = controller_state_msg_.error.joints[i].effort;
	}
/* TODO source node may not publish these states
	// Setting the contact states
	unsigned int num_contacts = controller_state_msg_.actual.contacts.size();
	for (unsigned int i = 0; i < num_contacts; i++) {
		// Getting the contact message
		dwl_msgs::ContactState contact_msg = robot_state_msg_.contacts[i];

		std::string name = contact_msg.name;
		dwl::urdf_model::LinkID contact_links = system_.getEndEffectors();

		// Updating the contact position
		Eigen::VectorXd position(3);
		position << contact_msg.position.x, contact_msg.position.y, contact_msg.position.z;
		robot_state.contact_pos[name] = position;

		// Updating the contact velocity
		Eigen::VectorXd velocity(3);
		velocity << contact_msg.velocity.x, contact_msg.velocity.y, contact_msg.velocity.z;
		robot_state.contact_vel[name] = velocity;

		// Updating the contact acceleration
		Eigen::VectorXd acceleration(3);
		acceleration << contact_msg.acceleration.x, contact_msg.acceleration.y, contact_msg.acceleration.z;
		robot_state.contact_acc[name] = acceleration;

		// Updating the contact wrench
		dwl::rbd::Vector6d effort;
		effort(dwl::rbd::LX) = contact_msg.wrench.force.x;
		effort(dwl::rbd::LY) = contact_msg.wrench.force.y;
		effort(dwl::rbd::LZ) = contact_msg.wrench.force.z;
		effort(dwl::rbd::AX) = contact_msg.wrench.torque.x;
		effort(dwl::rbd::AY) = contact_msg.wrench.torque.y;
		effort(dwl::rbd::AZ) = contact_msg.wrench.torque.z;
		robot_state.contact_eff[name] = effort;
	}*/
}


bool PlannerCommons::getRobotState(dwl::WholeBodyState& robot_state)
{
	if (received_robot_state_) {
		updateRobotStateSubscription(robot_state);
		received_robot_state_ = false;

		return true;
	} else
		return false;
}


bool PlannerCommons::getControllerState(dwl::WholeBodyState& desired,
										dwl::WholeBodyState& actual,
										dwl::WholeBodyState& error)
{
	if (received_controller_state_) {
		updateControllerStateSubscription(desired, actual, error);
		received_controller_state_ = false;

		return true;
	} else
		return false;
}


void PlannerCommons::setRobotStateCB(const dwl_msgs::WholeBodyStateConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	robot_state_buffer_.writeFromNonRT(*msg);

	robot_state_msg_ = *robot_state_buffer_.readFromNonRT();

	received_robot_state_ = true;
}


void PlannerCommons::setControllerStateCB(const dwl_msgs::WholeBodyControllerConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	controller_state_buffer_.writeFromNonRT(*msg);

	controller_state_msg_ = *controller_state_buffer_.readFromNonRT();

	received_controller_state_ = true;
}


} //@namespace dwl_msgs
