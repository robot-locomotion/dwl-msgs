#ifndef DWL_MSGS__WHOLE_BODY_STATE_INTERFACE__H
#define DWL_MSGS__WHOLE_BODY_STATE_INTERFACE__H

#include <dwl/model/FloatingBaseSystem.h>

// Messages headers
#include <dwl_msgs/WholeBodyState.h>
#include <dwl_msgs/WholeBodyTrajectory.h>


namespace dwl_msgs
{

/**
 * @brief The WholeBodyStateInterface class
 * This class provides a interface between the WholeBodyState ROS messages
 * (dwl_msgs::WholeBodyState) and the WholeBodyState class (dwl::WholeBodyState)
 */
class WholeBodyStateInterface
{
	public:
		/** @brief Constructor function */
		WholeBodyStateInterface();
		WholeBodyStateInterface(const dwl::model::FloatingBaseSystem& fbs);

		/** @brief Destructor function */
		~WholeBodyStateInterface();

		/**
		 * @brief Resets the floating-base system information
		 * @param[in] fbs Floating-base system
		 */
		void reset(const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Writes a dwl_msgs::WholeBodyState from a dwl::WholeBodyState
		 * @param[out] msg Whole-body state ros message
		 * @param[in] state Whole-body state
		 */
		void writeToMessage(dwl_msgs::WholeBodyState& msg,
							const dwl::WholeBodyState& state);

		/**
		 * @brief Writes a dwl_msgs::WholeBodyTrajectory from a
		 * dwl::WholeBodyTrajectory
		 * @param[out] msg Whole-body trajectory ros message
		 * @param[in] traj Whole-body trajectory
		 */
		void writeToMessage(dwl_msgs::WholeBodyTrajectory& msg,
							const dwl::WholeBodyTrajectory& traj);

		/**
		 * @brief Writes a dwl::WholeBodyState from a dwl_msgs::WholeBodyState
		 * @param[out] state Whole-body state ros message
		 * @param[in] msg Whole-body state
		 */
		void writeFromMessage(dwl::WholeBodyState& state,
							  const dwl_msgs::WholeBodyState& msg);

		/**
		 * @brief Writes a dwl::WholeBodyTrajectory from a
		 * dwl_msgs::WholeBodyTrajectory
		 * @param[out] traj Whole-body trajectory ros message
		 * @param[in] msg Whole-body trajectory
		 */
		void writeFromMessage(dwl::WholeBodyTrajectory& traj,
							  const dwl_msgs::WholeBodyTrajectory& msg);

		void writetoMessage(geometry_msgs::Pose& msg,
							const dwl::SE3& state);
		void writetoMessage(geometry_msgs::Twist& msg,
							const dwl::Motion& state);
		void writetoMessage(geometry_msgs::Wrench& msg,
							const dwl::Force& state);
		void writeFromMessage(dwl::SE3& state,
							  const geometry_msgs::Pose& msg);
		void writeFromMessage(dwl::Motion& state,
							  const geometry_msgs::Twist& msg);
		void writeFromMessage(dwl::Force& state,
							  geometry_msgs::Wrench& msg);


	private:
		/** @brief the floating-base system information */
		std::shared_ptr<dwl::model::FloatingBaseSystem> fbs_;

		/** @brief Indicates if it was defined the floating-base system */
		bool is_system_;
};

} //@namespace dwl_msgs

#endif
