#ifndef DWL_MSGS__PLANNER_COMMONS__H
#define DWL_MSGS__PLANNER_COMMONS__H

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>

#include <dwl_msgs/WholeBodyStateInterface.h>
#include <dwl_msgs/ReducedBodyStateInterface.h>

// Messages headers
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
		 * @param[in] node_pub ROS node handle used by the publishing
		 * @param[in] node_params ROS node handle used by the reading of params
		 * @param[in] fbs Floating-base system information
		 */
		void initMotionPlanStatePublisher(ros::NodeHandle node_pub,
										  ros::NodeHandle node_params,
										  const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Created a publisher of the reduced-motion plan. The name
		 * of the topic is defined as node_ns/reduced_plan
		 * @param[in] node_pub ROS node handle used by the publishing
		 * @param[in] node_params ROS node handle used by the reading of params
		 */
		void initReducedPlanPublisher(ros::NodeHandle node_pub,
									  ros::NodeHandle node_params);

		/**
		 * @brief Creates a subscriber of the robot state. The name of the
		 * topic is defined as node_ns/robot_states
		 * @param[in] node ROS node handle used by the subscription
		 * @param[in] fbs Floating-base system information
		 */
		void initRobotStateSubscriber(ros::NodeHandle node,
									  const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Creates a subscriber of the controller state. The name of the
		 * topic is defined as node_ns/state
		 * @param[in] node ROS node handle used by the subscription
		 * @param[in] fbs Floating-base system information
		 */
		void initControllertStateSubscriber(ros::NodeHandle node,
											const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Publishes the motion plan messages
		 * @param[in] current_state Current whole-body state
		 * @param[in] trajectory Planned whole-body trajectory
		 */
		void publishMotionPlan(const dwl::WholeBodyState& current_state,
							   const dwl::WholeBodyTrajectory& trajectory);

		/**
		 * @brief Publishes the reduced motion plan messages
		 * @param[in] current_state Current reduced-body state
		 * @param[in] trajectory Planned reduced-body state
		 */
		void publishReducedPlan(const dwl::ReducedBodyState& current_state,
								const dwl::ReducedBodyTrajectory& trajectory);

		/**
		 * @brief Updates the robot state subscription status which is
		 * real-time friendly. This provides us the current robot states
		 * @param[out] robot_state Whole-body state
		 */
		void updateRobotStateSubscription(dwl::WholeBodyState& robot_state);

		/**
		 * @brief Updates the controller state subscription status which is
		 * real-time friendly. This provides us the current robot states
		 * @param[out] desired Desired whole-body state
		 * @param[out] actual Actual whole-body state
		 * @param[out] error Error whole-body state
		 */
		void updateControllerStateSubscription(dwl::WholeBodyState& desired,
											   dwl::WholeBodyState& actual,
											   dwl::WholeBodyState& error);

		/**
		 * @brief Gets the robot state if there is a received message
		 * @param[out] robot_state Received robot state
		 * @return Returns true if there is a received robot state
		 */
		bool getRobotState(dwl::WholeBodyState& robot_state);

		/**
		 * @brief Gets the controller state if there is a received message
		 * @param[out] desired Desired whole-body state
		 * @param[out] actual Actual whole-body state
		 * @param[out] error Error whole-body state
		 * @return Returns true if there is a received controller state
		 */
		bool getControllerState(dwl::WholeBodyState& desired,
								dwl::WholeBodyState& actual,
								dwl::WholeBodyState& error);


	private:
		/** @brief Robot state and trajectory message interfaces */
		dwl_msgs::WholeBodyStateInterface wb_iface_;
		dwl_msgs::ReducedBodyStateInterface rb_iface_;

		/**
		 * @brief Robot state callback function of the subscriber
		 * @param[in] msg Whole-body state message
		 */
		void setRobotStateCB(const dwl_msgs::WholeBodyStateConstPtr& msg);


		/**
		 * @brief Controller state callback function of the subscriber
		 * @param[in] msg Whole-body state message
		 */
		void setControllerStateCB(const dwl_msgs::WholeBodyControllerConstPtr& msg);

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
