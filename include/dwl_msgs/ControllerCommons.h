#ifndef DWL_MSGS__CONTROLLER_COMMONS__H
#define DWL_MSGS__CONTROLLER_COMMONS__H

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <dwl_msgs/WholeBodyStateInterface.h>
#include <dwl/utils/DynamicLocomotion.h>

// Message headers
#include <dwl_msgs/WholeBodyController.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace dwl_msgs
{

class ControllerCommons
{
	struct ImuData
	{
		Eigen::Quaterniond orientation;
		Eigen::Vector3d angular_velocity;
		Eigen::Vector3d linear_acceleration;
	};

	public:
		/** @brief Constructor function */
		ControllerCommons();

		/** @brief Destructor function */
		~ControllerCommons();

		/**
		 * @brief Creates a real-time friendly publisher of the controller
		 * state. The name of the topic is defined as node_ns/state
		 * @param[in] node ROS node handle used by the subscription
		 * @param[in] fbs Floating-base system information
		 */
		void initControllerStatePublisher(ros::NodeHandle node,
										  const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Creates a real-time friendly publisher of the whole-body
		 * state. The controller parameters are read in the namespace node_ns.
		 * On the other hand, the name of the topic where is published the
		 * message is defined as node_parent_ns/robot_states
		 * @param[in] node ROS node handle used by the subscription
		 * @param[in] fbs Floating-base system information
		 */
		void initWholeBodyStatePublisher(ros::NodeHandle node,
										 const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Creates a real-time friendly publisher of the base state
		 * estimation state. The name of the topic is defined as node_ns/odom
		 * @param[in] node ROS node handle used by the subscription
		 */
		void initStateEstimationPublisher(ros::NodeHandle node);

		/**
		 * @brief Creates a real-time friendly publisher of the Imu state.
		 * The name of the topic is defined as node_ns/imu
		 */
		void initImuPublisher(ros::NodeHandle node,
							  std::string imu_frame);

		/**
		 * @brief Creates a real-time subscriber of base state estimation.
		 * The name of the topic is defined as node_ns/odom
		 * @param[in] node ROS node handle used by the subscription
		 */
		void initStateEstimationSubscriber(ros::NodeHandle node);

		/**
		 * @brief Creates a real-time subscriber of motion plans. The name of
		 * the topic is defined as node_ns/plan
		 * @param[in] node ROS node handle used by the subscription
		 * @param[in] fbs Floating-base system information
		 */
		void initMotionPlanSubscriber(ros::NodeHandle node,
									  const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Publishes the controller state
		 * @param[in] time Current time to be published the message
		 * @param[in] current_state Whole-body current state
		 * @param[in] desired_state Whole-body desired state
		 * @param[in] error_state Whole-body error state
		 * @param[in] feedforward_cmd Feedforward command
		 * @param[in] feedback_cmd Feedback command
		 * @param[in] cmd_eff Total command
		 */
		void publishControllerState(const ros::Time& time,
									const dwl::WholeBodyState& current_state,
									const dwl::WholeBodyState& desired_state,
									const dwl::WholeBodyState& error_state,
									const Eigen::VectorXd& feedforward_cmd,
									const Eigen::VectorXd& feedback_cmd,
									const Eigen::VectorXd& command_eff);

		/**
		 * @brief Publishes the whole-body state
		 * @param[in] time Current time to be published the message
		 * @param[in] state Current whole-body state to be
		 * published the message
		 */
		void publishWholeBodyState(const ros::Time& time,
								   const dwl::WholeBodyState& state);

		/**
		 * @brief Publishes the base state estimation messages which is
		 * real-time friendly
		 * @param[in] time Current time to be published the message
		 * @param[in] base_pos Current base position
		 * @param[in] base_vel Current base velocity
		 */
		void publishStateEstimation(const ros::Time& time,
									const Eigen::Vector6d& base_pos,
									const Eigen::Vector6d& base_vel);

		/**
		 * @brief Publishes the Imu state which is real-time friendly
		 * @param[in] time Current time to be published the message
		 * @param[in] imu Current Imu data
		 */
		void publishImuState(const ros::Time& time,
							 const struct ImuData& imu);

		/**
		 * @brief Updates the base state estimation subscription status which
		 * is real-time friendly. This provides us the current base states
		 * @param[out] base_pos Estimated base position
		 * @param[out] base_vel Estimated base velocity
		 */
		void updateStateEstimationSubscription(dwl::SE3& base_pos,
											   dwl::Motion& base_vel);

		/**
		 * @brief Updates the motion plan subscription status which is
		 * real-time friendly. This provides us the desired robot states
		 * @param[out] state Whole-body desired state
		 */
		void updateMotionPlanSubscription(dwl::WholeBodyState& state);

		/**
		 * @brief Sets required initial values for updating the plan once a
		 * new motion plan is received
		 * @param[in] plan Motion plan
		 */
		void setPlan(const dwl_msgs::WholeBodyTrajectory& plan);

		/**
		 * @brief Updates the desired robot states give a motion plan.
		 * A motion plan is described with a desired robot trajectory.
		 * This motion plan is updated once the trajectory is finished.
		 * @param[ou] state Whole-body desired state
		 */
		void updatePlan(dwl::WholeBodyState& state);


	private:
		/** @brief Robot state message interfaces */
		dwl_msgs::WholeBodyStateInterface wb_iface_;

		/**
		 * @brief Base state estimation callback function of the subscriber
		 * @param const nav_msgs::OdometryConstPtr& State estimation message
		 */
		void setBaseStateCB(const nav_msgs::OdometryConstPtr& msg);

		/**
		 * @brief Motion plan callback function of the subscriber
		 * @param const dwl_msgs::WholeBodyTrajectoryConsttPtr& Plan message
		 */
		void setPlanCB(const dwl_msgs::WholeBodyTrajectoryConstPtr& msg);

		/** @brief the floating-base system information */
		std::shared_ptr<dwl::model::FloatingBaseSystem> fbs_;

		/** @brief Controller state publisher */
		boost::shared_ptr<realtime_tools::RealtimePublisher<dwl_msgs::WholeBodyController> > controller_state_pub_;

		/** @brief Controller state publisher */
		boost::shared_ptr<realtime_tools::RealtimePublisher<dwl_msgs::WholeBodyState> > robot_state_pub_;

		/** @brief State estimation publisher */
		boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > base_state_pub_;

        /** @brief Pose with covariance publisher */
        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped> > pose_covariance_pub_;

		/** @brief Imu state publisher */
		boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu> > imu_state_pub_;

		/** @brief Base state estimation message */
		nav_msgs::Odometry base_state_;

		/** @brief Odom TF publisher */
		boost::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> > tf_pub_;

		/** @brief Base state estimation subscriber */
		ros::Subscriber base_state_sub_;

		/** @brief Realtime buffer for the base state estimation  */
		realtime_tools::RealtimeBuffer<nav_msgs::Odometry> base_state_buffer_;

		/** @brief Indicates the initial base state estimation message is received */
		bool init_base_state_;

		/** @brief Base state position offset */
		Eigen::Vector6d base_state_offset_;

		/** @brief Motion plan */
		dwl_msgs::WholeBodyTrajectory plan_;

		/** @brief Motion plan subscriber */
		ros::Subscriber plan_sub_;

		/** @brief Realtime buffer for the motion plan message */
		realtime_tools::RealtimeBuffer<dwl_msgs::WholeBodyTrajectory> plan_buffer_;

		/** @brief Indicates if there is a new motion plan available */
		bool new_plan_;

		/** @brief Number of trajectory points of the motion plan */
		unsigned int num_traj_points_;

		/** @brief A trajectory counter used for updating the plan */
		unsigned int trajectory_counter_;

		/** @brief Controller state publish rate */
		double controller_publish_rate_;

		/** @brief Whole-body state publish rate */
		double robot_publish_rate_;

		/** @brief State estimation publish rate */
		double odom_publish_rate_;

		/** @brief Imu state publish rate */
		double imu_publish_rate_;

		/** @brief Last publishing time of the controller state */
		ros::Time last_controller_publish_time_;

		/** @brief Last publishing time of the robot state */
		ros::Time last_robot_publish_time_;

		/** @brief Last publishing time of the state estimation state */
		ros::Time last_odom_publish_time_;

		/** @brief Last publishing time of the Imu state */
		ros::Time last_imu_publish_time_;

		/** @brief Label that indicates that controller state publisher is initialized */
		bool init_controller_state_pub_;

		/** @brief Label that indicates that whole-body state publisher is initialized */
		bool init_robot_state_pub_;

		/** @brief Label that indicates that state estimation publisher is initialized */
		bool init_odom_state_pub_;

		/** @brief Label that indicates that imu publisher is initialized */
		bool init_imu_state_pub_;
};

} //@namespace dwl_msgs


#endif
