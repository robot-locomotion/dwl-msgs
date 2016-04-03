#ifndef DWL_MSGS__PLANNER_COMMONS__H
#define DWL_MSGS__PLANNER_COMMONS__H

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>

#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/locomotion/WholeBodyTrajectoryOptimization.h>
#include <dwl/utils/RigidBodyDynamics.h>

// Messages headers
#include <dwl_msgs/WholeBodyState.h>
#include <dwl_msgs/WholeBodyTrajectory.h>
#include <dwl_msgs/ReducedTrajectory.h>


namespace dwl_msgs
{

class PlannerCommons
{
	public:
		/** @brief Constructor function */
		PlannerCommons();

		/** @brief Destructor function */
		~PlannerCommons();

		/**
		 * @brief Creates a publisher of the motion plan. The name of the
		 * topic is defined as node_ns/plan
		 * @param ros::NodeHandle ROS node handle used by the publishing
		 * @param ros::NodeHandle ROS node handle used by the reading of params
		 * @param dwl::model::FloatingBaseSystem& Floating-base system information
		 */
		void initMotionPlanStatePublisher(ros::NodeHandle node_pub,
										  ros::NodeHandle node_params,
										  dwl::model::FloatingBaseSystem& system);

		/**
		 * @brief Created a publisher of the reduced-motion plan. The name
		 * of the topic is defined as node_ns/reduced_plan
		 * @param ros::NodeHandle ROS node handle used by the publishing
		 * @param ros::NodeHandle ROS node handle used by the reading of params
		 */
		void initReducedPlanPublisher(ros::NodeHandle node_pub,
									  ros::NodeHandle node_params);

		/**
		 * @brief Creates a subscriber of the robot state. The name of the
		 * topic is defined as node_ns/robot_states
		 * @param ros::NodeHandle ROS node handle used by the subscription
		 * @param dwl::model::FloatingBaseSystem& Floating-base system information
		 */
		void initRobotStateSubscriber(ros::NodeHandle node,
									  dwl::model::FloatingBaseSystem& system);

		/**
		 * @brief Publishes the motion plan messages
		 * @param const dwl::WholeBodyState& Current whole-body state
		 * @param const dwl::WholeBodyTrajectory& Planned whole-body trajectory
		 */
		void publishMotionPlan(const dwl::WholeBodyState& current_state,
							   const dwl::WholeBodyTrajectory& trajectory);

		/**
		 * @brief Publishes the reduced motion plan messages
		 * @param const dwl::ReducedTrajectory& Planned reduced-body state
		 */
		void publishReducedPlan(const dwl::ReducedBodyTrajectory& trajectory);

		/**
		 * @brief Updates the robot state subscription status which is
		 * real-time friendly. This provides us the current robot states
		 * @param dwl::WholeBodyState& Whole-body state
		 */
		void updateRobotStateSubscription(dwl::WholeBodyState& robot_state);

		/**
		 * @brief Gets the robot state if there is a received message
		 * @param dwl::WholeBodyState& Received robot state
		 * @return Returns true if there is a received robot state
		 */
		bool getRobotState(dwl::WholeBodyState& robot_state);


	private:
		/**
		 * @brief Writes the whole-body state message from a locomotion state
		 * @param dwl_msgs::WholeBodyState& Whole-body state message
		 * @param const dwl::WholeBodyState& Whole-body state
		 */
		void writeWholeBodyStateMessage(dwl_msgs::WholeBodyState& msg,
	 	 	 	 	 					const dwl::WholeBodyState& state);

		/**
		 * @brief Robot state callback function of the subscriber
		 * @param const dwl_msgs::WholeBodyStateConstPtr& Whole-body state message
		 */
		void setRobotStateCB(const dwl_msgs::WholeBodyStateConstPtr& msg);


		/** @brief the floating-base system information */
		dwl::model::FloatingBaseSystem system_;

		/** @brief Motion plan publisher */
		ros::Publisher motion_plan_pub_;

		/** @brief Reduced-motion plan publisher */
		ros::Publisher reduced_plan_pub_;

		/** @brief Whole-body trajectory message */
		dwl_msgs::WholeBodyTrajectory motion_plan_msg_;

		/** @brief Reduced-motion plan message */
		dwl_msgs::ReducedTrajectory reduced_plan_msg_;

		/** @brief Robot state message */
		dwl_msgs::WholeBodyState robot_state_msg_;

		/** @brief Robot state subscriber */
		ros::Subscriber robot_state_sub_;

		/** @brief Realtime buffer for the base state estimation  */
		realtime_tools::RealtimeBuffer<dwl_msgs::WholeBodyState> robot_state_buffer_;

		/** @brief Label that indicates if motion plan publisher is initialized */
		bool init_motion_plan_pub_;

		/** @brief Label that indicates if the stability trajectory publisher
		 * was initialized */
		bool init_reduced_plan_pub_;

		/** @brief World frame name */
		std::string world_frame_id_;

		/** @brief Label that indicates that robot state has been received */
		bool received_robot_state_;
};

} //@namespace dwl_msgs

#endif
