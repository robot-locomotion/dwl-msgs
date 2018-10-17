#include <dwl_msgs/ReducedBodyStateInterface.h>


namespace dwl_msgs
{

ReducedBodyStateInterface::ReducedBodyStateInterface() : is_system_(false)
{

}


ReducedBodyStateInterface::ReducedBodyStateInterface(dwl::model::FloatingBaseSystem& fbs) : is_system_(true)
{
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
}


void ReducedBodyStateInterface::reset(const dwl::model::FloatingBaseSystem& fbs)
{
	fbs_ = std::make_shared<dwl::model::FloatingBaseSystem>(fbs);
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
	const Eigen::Vector3d& com = state.getCoMSE3().getTranslation();
	msg.center_of_mass.x = com(dwl::rbd::X);
	msg.center_of_mass.y = com(dwl::rbd::Y);
	msg.center_of_mass.z = com(dwl::rbd::Z);

	// Filling the CoP position
	const Eigen::Vector3d& cop = state.getCoPPosition_W();
	msg.center_of_pressure.x = cop(dwl::rbd::X);
	msg.center_of_pressure.y = cop(dwl::rbd::Y);
	msg.center_of_pressure.z = cop(dwl::rbd::Z);

	// Filling the support position
	msg.support_region.resize(state.support_region.size());
	unsigned int idx = 0;
	for (dwl::SE3Map::const_iterator it = state.support_region.begin();
			it != state.support_region.end(); ++it) {
		dwl::SE3 vertex = it->second;
		msg.support_region[idx].x = vertex.getTranslation()(dwl::rbd::X);
		msg.support_region[idx].y = vertex.getTranslation()(dwl::rbd::Y);
		msg.support_region[idx].z = vertex.getTranslation()(dwl::rbd::Z);
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
	state.setCoMSE3(dwl::SE3(Eigen::Vector3d(msg.center_of_mass.x,
											 msg.center_of_mass.y,
											 msg.center_of_mass.z),
							 Eigen::Vector3d(0., 0., 0.)));

	// Filling the CoP position
	state.setCoPPosition_W(Eigen::Vector3d(msg.center_of_pressure.x,
										   msg.center_of_pressure.y,
										   msg.center_of_pressure.z));

	// Filling the support position
	Eigen::Vector3d vertex;
	for (unsigned int i = 0; i < msg.support_region.size(); ++i) {
		std::string name = fbs_->getEndEffectorList(dwl::model::FOOT)[i];
		vertex(dwl::rbd::X) = msg.support_region[i].x;
		vertex(dwl::rbd::Y) = msg.support_region[i].y;
		vertex(dwl::rbd::Z) = msg.support_region[i].z;

		state.setSupportRegion(name, dwl::SE3(vertex, Eigen::Vector3d(0.,0.,0.)));
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
