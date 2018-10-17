#include <dwl_msgs/WholeBodyStateInterface.h>


namespace dwl_msgs
{

WholeBodyStateInterface::WholeBodyStateInterface() : is_system_(false)
{

}


WholeBodyStateInterface::WholeBodyStateInterface(const dwl::model::FloatingBaseSystem& fbs) : is_system_(true)
{
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
}


void WholeBodyStateInterface::reset(const dwl::model::FloatingBaseSystem& fbs)
{
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
	is_system_ = true;
}


WholeBodyStateInterface::~WholeBodyStateInterface()
{

}


void WholeBodyStateInterface::writeToMessage(dwl_msgs::WholeBodyState& msg,
											 const dwl::WholeBodyState& state)
{
	if (!is_system_)
		printf(YELLOW "Warning: you cannot write the dwl_msg::WholeBodyState "
				"because it wasn't define the FloatingBaseSystem\n" COLOR_RESET);

	// Filling the time information
	msg.time = state.time;

	// Filling the base state
	if (fbs_->isFloatingBase()) {
		// Filling the data from the base state
		msg.base.name = fbs_->getFloatingBaseName();
		writetoMessage(msg.base.pose, state.getBaseSE3());
		writetoMessage(msg.base.velocity, state.getBaseVelocity_W());
		writetoMessage(msg.base.acceleration, state.getBaseAcceleration_W());
	}

	// Filling the joint state if there is information
	if (state.joint_pos.size() == fbs_->getJointDoF()) { //safety check
		msg.joints.resize(fbs_->getJointDoF());
		for (dwl::model::ElementId::const_iterator jnt_it = fbs_->getJoints().begin();
				jnt_it != fbs_->getJoints().end(); ++jnt_it) {
			std::string name = jnt_it->first;
			unsigned int id = jnt_it->second;

			msg.joints[id].name = name;
			msg.joints[id].position = state.joint_pos(id);
			msg.joints[id].velocity = state.joint_vel(id);
			msg.joints[id].acceleration = state.joint_acc(id);
			msg.joints[id].effort = state.joint_eff(id);
		}
	}

	// Filling the contact state if there is information
	// Note that planners can(not) generates contact information, at the same
	// time, controllers could(not) used it. If the controller need a missing
	// contact information, then it could computed from the joint trajectory
	unsigned int counter = 0;
	dwl::model::ElementId contact_links = fbs_->getEndEffectors();
	msg.contacts.resize(contact_links.size());
	for (dwl::model::ElementId::const_iterator it = contact_links.begin();
			it != contact_links.end(); ++it) {
		std::string name = it->first;

		// Filling the data from the contact state
		msg.contacts[counter].name = name;
		writetoMessage(msg.contacts[counter].pose, state.getContactSE3_B(name));
		writetoMessage(msg.contacts[counter].velocity, state.getContactVelocity_B(name));
		writetoMessage(msg.contacts[counter].acceleration, state.getContactAcceleration_B(name));
		writetoMessage(msg.contacts[counter].wrench, state.getContactWrench_B(name));

		// Incrementing the contact counter
		++counter;
	}
}


void WholeBodyStateInterface::writeToMessage(dwl_msgs::WholeBodyTrajectory& msg,
											 const dwl::WholeBodyTrajectory& traj)
{
	// Filling the trajectory
	unsigned int num_points = traj.size();
	msg.trajectory.resize(num_points);
	for (unsigned int i = 0; i < num_points; ++i)
		writeToMessage(msg.trajectory[i], traj[i]);
}


void WholeBodyStateInterface::writeFromMessage(dwl::WholeBodyState& state,
					  	  	  	  	  	  	   const dwl_msgs::WholeBodyState& msg)
{
	if (!is_system_)
		printf(YELLOW "Warning: you cannot write the dwl::WholeBodyState "
				"because it wasn't define the FloatingBaseSystem\n" COLOR_RESET);

	// Writing the time information
	state.time = msg.time;

	// Writing the base states
	writeFromMessage(state.base_pos, msg.base.pose);
	writeFromMessage(state.base_vel, msg.base.velocity);
	writeFromMessage(state.base_acc, msg.base.acceleration);

	// Sanity check: checking the size of the joint states
	if (state.joint_pos.size() != fbs_->getJointDoF())
		state.joint_pos = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	if (state.joint_vel.size() != fbs_->getJointDoF())
		state.joint_vel = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	if (state.joint_acc.size() != fbs_->getJointDoF())
		state.joint_acc = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	if (state.joint_eff.size() != fbs_->getJointDoF())
		state.joint_eff = Eigen::VectorXd::Zero(fbs_->getJointDoF());

	// Writing the joint states
	unsigned num_joints = msg.joints.size();
	for (unsigned int i = 0; i < num_joints; ++i) {
		std::string name = msg.joints[i].name;
		unsigned int joint_id = fbs_->getJointId(name);

		state.joint_pos(joint_id) = msg.joints[i].position;
		state.joint_vel(joint_id) = msg.joints[i].velocity;
		state.joint_acc(joint_id) = msg.joints[i].acceleration;
		state.joint_eff(joint_id) = msg.joints[i].effort;
	}

	// Writing the contact states
//	Eigen::Vector3d contact_state;
	unsigned int num_contacts = msg.contacts.size();
	for (unsigned int i = 0; i < num_contacts; ++i) {
		// Getting the contact message
		dwl_msgs::ContactState contact_msg = msg.contacts[i];

		// Getting the contact name
		std::string name = contact_msg.name;

		dwl::SE3 pose;
		dwl::Motion vel, acc;
		dwl::Force wrch;

		// Getting the data from the message
		writeFromMessage(pose, contact_msg.pose);
		writeFromMessage(vel, contact_msg.velocity);
		writeFromMessage(acc, contact_msg.acceleration);
		writeFromMessage(wrch, contact_msg.wrench);

		// Filling the data inside the whole-body state
		state.setContactSE3_B(name, pose);
		state.setContactVelocity_B(name, vel);
		state.setContactAcceleration_B(name, acc);
		state.setContactWrench_B(name, wrch);
	}
}


void WholeBodyStateInterface::writeFromMessage(dwl::WholeBodyTrajectory& traj,
											   const dwl_msgs::WholeBodyTrajectory& msg)
{
	// Filling the trajectory
	unsigned int num_points = msg.trajectory.size();
	traj.resize(num_points);
	for (unsigned int i = 0; i < num_points; ++i)
		writeFromMessage(traj[i], msg.trajectory[i]);
}


void WholeBodyStateInterface::writetoMessage(geometry_msgs::Pose& msg,
											 const dwl::SE3& state)
{
	const Eigen::Vector7d& pose = state.toVector();

	// Filling the pose
	msg.position.x = pose(dwl::rbd::LX_Q);
	msg.position.y = pose(dwl::rbd::LY_Q);
	msg.position.z = pose(dwl::rbd::LZ_Q);
	msg.orientation.x = pose(dwl::rbd::AX_Q);
	msg.orientation.y = pose(dwl::rbd::AY_Q);
	msg.orientation.z = pose(dwl::rbd::AZ_Q);
	msg.orientation.w = pose(dwl::rbd::AW_Q);
}


void WholeBodyStateInterface::writetoMessage(geometry_msgs::Twist& msg,
											 const dwl::Motion& state)
{
	const Eigen::Vector6d& twist = state.toVector();

	// Filling the pose
	msg.linear.x = twist(dwl::rbd::LX_V);
	msg.linear.y = twist(dwl::rbd::LY_V);
	msg.linear.z = twist(dwl::rbd::LZ_V);
	msg.angular.x = twist(dwl::rbd::AX_V);
	msg.angular.y = twist(dwl::rbd::AY_V);
	msg.angular.z = twist(dwl::rbd::AZ_V);
}


void WholeBodyStateInterface::writetoMessage(geometry_msgs::Wrench& msg,
											 const dwl::Force& state)
{
	const Eigen::Vector6d& wrench = state.toVector();

	// Filling the pose
	msg.force.x = wrench(dwl::rbd::LX_V);
	msg.force.y = wrench(dwl::rbd::LY_V);
	msg.force.z = wrench(dwl::rbd::LZ_V);
	msg.torque.x = wrench(dwl::rbd::AX_V);
	msg.torque.y = wrench(dwl::rbd::AY_V);
	msg.torque.z = wrench(dwl::rbd::AZ_V);
}


void WholeBodyStateInterface::writeFromMessage(dwl::SE3& state,
											   const geometry_msgs::Pose& msg)
{
	// Filling the message in the state
	state.setTranslation(Eigen::Vector3d(msg.position.x,
										 msg.position.y,
										 msg.position.z));
	state.setQuaternion(Eigen::Vector4d(msg.orientation.x,
										msg.orientation.y,
										msg.orientation.z,
										msg.orientation.w));
}


void WholeBodyStateInterface::writeFromMessage(dwl::Motion& state,
					  	  	  	  	  	  	   const geometry_msgs::Twist& msg)
{
	// Filling the message in the state
	state.setLinear(Eigen::Vector3d(msg.linear.x,
									msg.linear.y,
									msg.linear.z));
	state.setAngular(Eigen::Vector3d(msg.angular.x,
									 msg.angular.y,
									 msg.angular.z));
}


void WholeBodyStateInterface::writeFromMessage(dwl::Force& state,
					  	  	  	  	  	  	   geometry_msgs::Wrench& msg)
{
	// Filling the message in the state
	state.setLinear(Eigen::Vector3d(msg.force.x,
									msg.force.y,
									msg.force.z));
	state.setAngular(Eigen::Vector3d(msg.torque.x,
									 msg.torque.y,
									 msg.torque.z));
}

} //@namespace dwl_msgs
