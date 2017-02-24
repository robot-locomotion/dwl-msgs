#include <dwl_msgs/ReducedBodyStateInterface.h>


namespace dwl_msgs
{

ReducedBodyStateInterface::ReducedBodyStateInterface() : is_system_(false)
{

}


ReducedBodyStateInterface::ReducedBodyStateInterface(const dwl::model::FloatingBaseSystem& system) : is_system_(true)
{
	fbs_ = system;
}


void ReducedBodyStateInterface::reset(const dwl::model::FloatingBaseSystem& system)
{
	fbs_ = system;
	is_system_ = true;
}


ReducedBodyStateInterface::~ReducedBodyStateInterface()
{

}


void ReducedBodyStateInterface::writeToMessage(dwl_msgs::ReducedBodyState& msg,
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


void ReducedBodyStateInterface::writeToMessage(dwl_msgs::ReducedBodyTrajectory& msg,
											   const dwl::ReducedBodyTrajectory& traj)
{
	// Filling the trajectory
	unsigned int num_points = traj.size();
	msg.trajectory.resize(num_points);
	for (unsigned int i = 0; i < num_points; i++)
		writeToMessage(msg.trajectory[i], traj[i]);
}


void ReducedBodyStateInterface::writeFromMessage(dwl::ReducedBodyState& state,
					  	  	  	  	  	  		 const dwl_msgs::ReducedBodyState& msg)
{
	if (!is_system_)
		printf(YELLOW "Warning: you cannot write the dwl_msg::ReducedBodyState "
				"because it wasn't define the FloatingBaseSystem\n" COLOR_RESET);

	// Filling the time
	state.time = msg.time;

	// Filling the CoM position
	state.com_pos(dwl::rbd::X) = msg.center_of_mass.x;
	state.com_pos(dwl::rbd::Y) = msg.center_of_mass.y;
	state.com_pos(dwl::rbd::Z) = msg.center_of_mass.z;

	// Filling the CoP position
	state.cop(dwl::rbd::X) = msg.center_of_pressure.x;
	state.cop(dwl::rbd::Y) = msg.center_of_pressure.y;
	state.cop(dwl::rbd::Z) = msg.center_of_pressure.z;

	// Filling the support position
	Eigen::Vector3d vertex;
	for (unsigned int i = 0; i < msg.support_region.size(); i++) {
		std::string name = fbs_.getEndEffectorNames(dwl::model::FOOT)[i];
		vertex(dwl::rbd::X) = msg.support_region[i].x;
		vertex(dwl::rbd::Y) = msg.support_region[i].y;
		vertex(dwl::rbd::Z) = msg.support_region[i].z;

		state.support_region[name] = vertex;
	}
}


void ReducedBodyStateInterface::writeFromMessage(dwl::ReducedBodyTrajectory& traj,
												 const dwl_msgs::ReducedBodyTrajectory& msg)
{
	// Filling the trajectory
	unsigned int num_points = msg.trajectory.size();
	traj.resize(num_points);
	for (unsigned int i = 0; i < num_points; i++)
		writeFromMessage(traj[i], msg.trajectory[i]);
}

} //@namespace dwl_msgs
