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
	wb_iface_.reset(system);

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
	wb_iface_.reset(system);

	robot_state_sub_ = node.subscribe<dwl_msgs::WholeBodyState>(node.getNamespace() + "/robot_states", 1,
			&PlannerCommons::setRobotStateCB, this, ros::TransportHints().tcpNoDelay());
}


void PlannerCommons::initControllertStateSubscriber(ros::NodeHandle node,
													dwl::model::FloatingBaseSystem& system)
{
	// Setting the floating-base system info
	wb_iface_.reset(system);

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
		rb_iface_.writeToMessage(reduced_plan_msg_.actual, current_state);

		// Filling the trajectory
		rb_iface_.writeToMessage(reduced_plan_msg_, trajectory);

		// Publishing the motion plan
		reduced_plan_pub_.publish(reduced_plan_msg_);
	}
}


void PlannerCommons::updateRobotStateSubscription(dwl::WholeBodyState& robot_state)
{
	// Converting the actual robot state message to dwl::WholeBodyState
	wb_iface_.writeFromMessage(robot_state, robot_state_msg_);
}


void PlannerCommons::updateControllerStateSubscription(dwl::WholeBodyState& desired,
													   dwl::WholeBodyState& actual,
													   dwl::WholeBodyState& error)
{
	// Converting the desired, actual and error whole-body states
	wb_iface_.writeFromMessage(desired, controller_state_msg_.desired);
	wb_iface_.writeFromMessage(actual, controller_state_msg_.actual);
	wb_iface_.writeFromMessage(error, controller_state_msg_.error);
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
