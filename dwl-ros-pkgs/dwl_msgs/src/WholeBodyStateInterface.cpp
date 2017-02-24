#include <dwl_msgs/WholeBodyStateInterface.h>


namespace dwl_msgs
{

WholeBodyStateInterface::WholeBodyStateInterface() : is_system_(false)
{

}


WholeBodyStateInterface::WholeBodyStateInterface(const dwl::model::FloatingBaseSystem& system) : is_system_(true)
{
	fbs_ = system;
}


void WholeBodyStateInterface::reset(const dwl::model::FloatingBaseSystem& system)
{
	fbs_ = system;
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
	if (fbs_.isFullyFloatingBase())
		msg.base.resize(6);
	else
		msg.base.resize(fbs_.getFloatingBaseDoF());
	unsigned int counter = 0;
	for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
		dwl::rbd::Coords6d base_coord = dwl::rbd::Coords6d(base_idx);
		dwl::model::FloatingBaseJoint base_joint = fbs_.getFloatingBaseJoint(base_coord);

		if (base_joint.active) {
			msg.base[counter].id = base_idx;
			msg.base[counter].name = base_joint.name;

			msg.base[counter].position = state.base_pos(base_idx);
			msg.base[counter].velocity = state.base_vel(base_idx);
			msg.base[counter].acceleration = state.base_acc(base_idx);

			counter++;
		}
	}

	// Filling the joint state if there is information
	if (state.joint_pos.size() == fbs_.getJointDoF()) { //safety check
		msg.joints.resize(fbs_.getJointDoF());
		for (dwl::urdf_model::JointID::const_iterator jnt_it = fbs_.getJoints().begin();
				jnt_it != fbs_.getJoints().end(); jnt_it++) {
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
	unsigned int contact_counter = 0;
	msg.contacts.resize(state.contact_pos.size());
	dwl::urdf_model::LinkID contact_links = fbs_.getEndEffectors();
	for (dwl::urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
			contact_it != contact_links.end(); contact_it++) {
		std::string name = contact_it->first;

		// Defining the contact state vectors
		Eigen::Vector3d position, velocity, acceleration;
		dwl::rbd::Vector6d effort;

		// Defining the contact iterators
		dwl::rbd::BodyVectorXd::const_iterator pos_it, vel_it, acc_it;
		dwl::rbd::BodyVector6d::const_iterator eff_it;

		// Filling the contact state
		pos_it = state.contact_pos.find(name);
		vel_it = state.contact_vel.find(name);
		acc_it = state.contact_acc.find(name);
		eff_it = state.contact_eff.find(name);
		if (pos_it != state.contact_pos.end()) {
			// Filling the name of the contact
			msg.contacts[contact_counter].name = name;

			position = pos_it->second.tail(3);
			msg.contacts[contact_counter].position.x = position(dwl::rbd::X);
			msg.contacts[contact_counter].position.y = position(dwl::rbd::Y);
			msg.contacts[contact_counter].position.z = position(dwl::rbd::Z);

			if (vel_it != state.contact_vel.end()) {
				velocity = vel_it->second.tail(3);
				msg.contacts[contact_counter].velocity.x = velocity(dwl::rbd::X);
				msg.contacts[contact_counter].velocity.y = velocity(dwl::rbd::Y);
				msg.contacts[contact_counter].velocity.z = velocity(dwl::rbd::Z);
			}
			if (acc_it != state.contact_acc.end()) {
				acceleration = acc_it->second.tail(3);
				msg.contacts[contact_counter].acceleration.x = acceleration(dwl::rbd::X);
				msg.contacts[contact_counter].acceleration.y = acceleration(dwl::rbd::Y);
				msg.contacts[contact_counter].acceleration.z = acceleration(dwl::rbd::Z);
			}
			if (eff_it != state.contact_eff.end()) {
				effort = eff_it->second;
				msg.contacts[contact_counter].wrench.torque.x = effort(dwl::rbd::AX);
				msg.contacts[contact_counter].wrench.torque.y = effort(dwl::rbd::AY);
				msg.contacts[contact_counter].wrench.torque.z = effort(dwl::rbd::AZ);
				msg.contacts[contact_counter].wrench.force.x = effort(dwl::rbd::LX);
				msg.contacts[contact_counter].wrench.force.y = effort(dwl::rbd::LY);
				msg.contacts[contact_counter].wrench.force.z = effort(dwl::rbd::LZ);
			}

			// Incrementing the contact counter
			contact_counter++;
		}
	}
}


void WholeBodyStateInterface::writeToMessage(dwl_msgs::WholeBodyTrajectory& msg,
											 const dwl::WholeBodyTrajectory& traj)
{
	// Filling the trajectory
	unsigned int num_points = traj.size();
	msg.trajectory.resize(num_points);
	for (unsigned int i = 0; i < num_points; i++)
		writeToMessage(msg.trajectory[i], traj[i]);
}


void WholeBodyStateInterface::writeFromMessage(dwl::WholeBodyState& state,
					  	  	  	  	  	  	   const dwl_msgs::WholeBodyState& msg)
{
	if (!is_system_)
		printf(YELLOW "Warning: you cannot write the dwl::WholeBodyState "
				"because it wasn't define the FloatingBaseSystem\n" COLOR_RESET);

	// Writing the base states
	unsigned num_base = msg.base.size();
	for (unsigned int i = 0; i < num_base; i++) {
		unsigned base_id = msg.base[i].id;

		state.base_pos(base_id) = msg.base[i].position;
		state.base_vel(base_id) = msg.base[i].velocity;
		state.base_acc(base_id) = msg.base[i].acceleration;
	}

	// Sanity check: checking the size of the joint states
	if (state.joint_pos.size() != fbs_.getJointDoF())
		state.joint_pos = Eigen::VectorXd::Zero(fbs_.getJointDoF());
	if (state.joint_vel.size() != fbs_.getJointDoF())
		state.joint_vel = Eigen::VectorXd::Zero(fbs_.getJointDoF());
	if (state.joint_acc.size() != fbs_.getJointDoF())
		state.joint_acc = Eigen::VectorXd::Zero(fbs_.getJointDoF());
	if (state.joint_eff.size() != fbs_.getJointDoF())
		state.joint_eff = Eigen::VectorXd::Zero(fbs_.getJointDoF());

	// Writing the joint states
	dwl::urdf_model::JointID joints = fbs_.getJoints();
	unsigned num_joints = msg.joints.size();
	for (unsigned int i = 0; i < num_joints; i++) {
		std::string name = msg.joints[i].name;

		dwl::urdf_model::JointID::iterator joint_it = joints.find(name);
		unsigned int joint_id = joint_it->second;

		state.joint_pos(joint_id) = msg.joints[i].position;
		state.joint_vel(joint_id) = msg.joints[i].velocity;
		state.joint_acc(joint_id) = msg.joints[i].acceleration;
		state.joint_eff(joint_id) = msg.joints[i].effort;
	}

	// Writing the contact states
	unsigned int num_contacts = msg.contacts.size();
	for (unsigned int i = 0; i < num_contacts; i++) {
		// Getting the contact message
		dwl_msgs::ContactState contact_msg = msg.contacts[i];

		// Getting the contact name
		std::string name = contact_msg.name;

		// Updating the contact position
		Eigen::VectorXd position(3);
		position << contact_msg.position.x,
					contact_msg.position.y,
					contact_msg.position.z;
		state.contact_pos[name] = position;

		// Updating the contact velocity
		Eigen::VectorXd velocity(3);
		velocity << contact_msg.velocity.x,
					contact_msg.velocity.y,
					contact_msg.velocity.z;
		state.contact_vel[name] = velocity;

		// Updating the contact acceleration
		Eigen::VectorXd acceleration(3);
		acceleration << contact_msg.acceleration.x,
						contact_msg.acceleration.y,
						contact_msg.acceleration.z;
		state.contact_acc[name] = acceleration;

		// Updating the contact wrench
		dwl::rbd::Vector6d effort;
		effort(dwl::rbd::AX) = contact_msg.wrench.torque.x;
		effort(dwl::rbd::AY) = contact_msg.wrench.torque.y;
		effort(dwl::rbd::AZ) = contact_msg.wrench.torque.z;
		effort(dwl::rbd::LX) = contact_msg.wrench.force.x;
		effort(dwl::rbd::LY) = contact_msg.wrench.force.y;
		effort(dwl::rbd::LZ) = contact_msg.wrench.force.z;
		state.contact_eff[name] = effort;
	}
}


void WholeBodyStateInterface::writeFromMessage(dwl::WholeBodyTrajectory& traj,
											   const dwl_msgs::WholeBodyTrajectory& msg)
{
	// Filling the trajectory
	unsigned int num_points = msg.trajectory.size();
	traj.resize(num_points);
	for (unsigned int i = 0; i < num_points; i++)
		writeFromMessage(traj[i], msg.trajectory[i]);
}

} //@namespace dwl_msgs
