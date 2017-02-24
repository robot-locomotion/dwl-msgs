#ifndef DWL_MSGS__PLANNER_COMMONS__H
#define DWL_MSGS__PLANNER_COMMONS__H

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>

#include <dwl_msgs/WholeBodyStateInterface.h>
#include <dwl/locomotion/WholeBodyTrajectoryOptimization.h>

// Messages headers
#include <dwl_msgs/ReducedBodyState.h>
#include <dwl_msgs/ReducedBodyTrajectory.h>
#include <dwl_msgs/WholeBodyController.h>


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
		 * @brief Creates a subscriber of the controller state. The name of the
		 * topic is defined as node_ns/state
		 * @param ros::NodeHandle ROS node handle used by the subscription
		 * @param dwl::model::FloatingBaseSystem& Floating-base system information
		 */
		void initControllertStateSubscriber(ros::NodeHandle node,
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
		 * @param const dwl::ReducedBodyState& Current reduced-body state
		 * @param const dwl::ReducedTrajectory& Planned reduced-body state
		 */
		void publishReducedPlan(const dwl::ReducedBodyState& current_state,
								const dwl::ReducedBodyTrajectory& trajectory);

		/**
		 * @brief Updates the robot state subscription status which is
		 * real-time friendly. This provides us the current robot states
		 * @param dwl::WholeBodyState& Whole-body state
		 */
		void updateRobotStateSubscription(dwl::WholeBodyState& robot_state);

		/**
		 * @brief Updates the controller state subscription status which is
		 * real-time friendly. This provides us the current robot states
		 * @param dwl::WholeBodyState& Desired whole-body state
		 * @param dwl::WholeBodyState& Actual whole-body state
		 * @param dwl::WholeBodyState& Error whole-body state
		 */
		void updateControllerStateSubscription(dwl::WholeBodyState& desired,
											   dwl::WholeBodyState& actual,
											   dwl::WholeBodyState& error);

		/**
		 * @brief Gets the robot state if there is a received message
		 * @param dwl::WholeBodyState& Received robot state
		 * @return Returns true if there is a received robot state
		 */
		bool getRobotState(dwl::WholeBodyState& robot_state);

		/**
		 * @brief Gets the controller state if there is a received message
		 * @param dwl::WholeBodyState& Desired whole-body state
		 * @param dwl::WholeBodyState& Actual whole-body state
		 * @param dwl::WholeBodyState& Error whole-body state
		 * @return Returns true if there is a received controller state
		 */
		bool getControllerState(dwl::WholeBodyState& desired,
								dwl::WholeBodyState& actual,
								dwl::WholeBodyState& error);


	private:
		dwl_msgs::WholeBodyStateInterface wb_iface_;

		/**
		 * @brief Writes the reduced-body state message from dwl::ReducedBodyState
		 * @param dwl_msgs::ReducedBodyState& Reduced-body state message
		 * @param const dwl::ReducedBodyState& Reduced-body sate
		 */
		void writeReducedBodyStateMessage(dwl_msgs::ReducedBodyState& msg,
										  const dwl::ReducedBodyState& state);

		/**
		 * @brief Robot state callback function of the subscriber
		 * @param const dwl_msgs::WholeBodyStateConstPtr& Whole-body state message
		 */
		void setRobotStateCB(const dwl_msgs::WholeBodyStateConstPtr& msg);


		/**
		 * @brief Controller state callback function of the subscriber
		 * @param const dwl_msgs::WholeBodyControllerConstPtr& Whole-body state message
		 */
		void setControllerStateCB(const dwl_msgs::WholeBodyControllerConstPtr& msg);


		/** @brief the floating-base system information */
		dwl::model::FloatingBaseSystem system_;

		/** @brief Motion plan publisher */
		ros::Publisher motion_plan_pub_;

		/** @brief Reduced-motion plan publisher */
		ros::Publisher reduced_plan_pub_;

		/** @brief Whole-body trajectory message */
		dwl_msgs::WholeBodyTrajectory motion_plan_msg_;

		/** @brief Reduced-motion plan message */
		dwl_msgs::ReducedBodyTrajectory reduced_plan_msg_;

		/** @brief Robot state message */
		dwl_msgs::WholeBodyState robot_state_msg_;

		/** @brief Controller state message */
		dwl_msgs::WholeBodyController controller_state_msg_;

		/** @brief Robot state subscriber */
		ros::Subscriber robot_state_sub_;

		/** @brief Controller state subscriber */
		ros::Subscriber controller_state_sub_;

		/** @brief Realtime buffer for the whole-body state */
		realtime_tools::RealtimeBuffer<dwl_msgs::WholeBodyState> robot_state_buffer_;

		/** @brief Realtime buffer for the whole-body controller state */
		realtime_tools::RealtimeBuffer<dwl_msgs::WholeBodyController> controller_state_buffer_;

		/** @brief Label that indicates if motion plan publisher is initialized */
		bool init_motion_plan_pub_;

		/** @brief Label that indicates if the stability trajectory publisher
		 * was initialized */
		bool init_reduced_plan_pub_;

		/** @brief World frame name */
		std::string world_frame_id_;

		/** @brief Label that indicates that robot state has been received */
		bool received_robot_state_;

		/** @brief Label that indicates that controller state has been received */
		bool received_controller_state_;
};

} //@namespace dwl_msgs

#endif
