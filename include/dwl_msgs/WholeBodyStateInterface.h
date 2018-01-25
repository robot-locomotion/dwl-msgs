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
		WholeBodyStateInterface(const dwl::model::FloatingBaseSystem& system);

		/** @brief Destructor function */
		~WholeBodyStateInterface();

		/**
		 * @brief Resets the floating-base system information
		 * @param const dwl::model::FloatingBaseSystem& Floating-base system
		 */
		void reset(const dwl::model::FloatingBaseSystem& system);

		/**
		 * @brief Writes a dwl_msgs::WholeBodyState from a dwl::WholeBodyState
		 * @param dwl_msgs::WholeBodyState& Whole-body state ros message
		 * @param const dwl::WholeBodyState& Whole-body state
		 */
		void writeToMessage(dwl_msgs::WholeBodyState& msg,
							const dwl::WholeBodyState& state);

		/**
		 * @brief Writes a dwl_msgs::WholeBodyTrajectory from a
		 * dwl::WholeBodyTrajectory
		 * @param dwl_msgs::WholeBodyTrajectory& Whole-body trajectory ros message
		 * @param const dwl::WholeBodyTrajectory& Whole-body trajectory
		 */
		void writeToMessage(dwl_msgs::WholeBodyTrajectory& msg,
							const dwl::WholeBodyTrajectory& traj);

		/**
		 * @brief Writes a dwl::WholeBodyState from a dwl_msgs::WholeBodyState
		 * @param dwl_msgs::WholeBodyState& Whole-body state ros message
		 * @param const dwl::WholeBodyState& Whole-body state
		 */
		void writeFromMessage(dwl::WholeBodyState& state,
							  const dwl_msgs::WholeBodyState& msg);

		/**
		 * @brief Writes a dwl::WholeBodyTrajectory from a
		 * dwl_msgs::WholeBodyTrajectory
		 * @param dwl_msgs::WholeBodyTrajectory& Whole-body trajectory ros message
		 * @param const dwl::WholeBodyTrajectory& Whole-body trajectory
		 */
		void writeFromMessage(dwl::WholeBodyTrajectory& traj,
							  const dwl_msgs::WholeBodyTrajectory& msg);


	private:
		/** @brief the floating-base system information */
		dwl::model::FloatingBaseSystem fbs_;

		/** @brief Indicates if it was defined the floating-base system */
		bool is_system_;
};

} //@namespace dwl_msgs

#endif
