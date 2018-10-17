#ifndef DWL_MSGS__REDUCED_BODY_STATE_INTERFACE__H
#define DWL_MSGS__REDUCED_BODY_STATE_INTERFACE__H

#include <dwl/model/FloatingBaseSystem.h>

// Messages headers
#include <dwl_msgs/ReducedBodyState.h>
#include <dwl_msgs/ReducedBodyTrajectory.h>


namespace dwl_msgs
{

/**
 * @brief The ReducedBodyStateInterface class
 * This class provides a interface between the ReducedBodyState ROS messages
 * (dwl_msgs::ReducedBodyState) and the ReducedBodyState class (dwl::ReducedBodyState)
 */
class ReducedBodyStateInterface
{
	public:
		/** @brief Constructor function */
		ReducedBodyStateInterface();
		ReducedBodyStateInterface(dwl::model::FloatingBaseSystem& fbs);

		/** @brief Destructor function */
		~ReducedBodyStateInterface();

		/**
		 * @brief Resets the floating-base system information
		 * @param[in] fbs Floating-base system
		 */
		void reset(const dwl::model::FloatingBaseSystem& fbs);

		/**
		 * @brief Writes a dwl_msgs::ReducedBodyState from a dwl::ReducedBodyState
		 * @param[ou] msg Reduced-body state ros message
		 * @param[in] state Reduced-body state
		 */
		void writeToMessage(dwl_msgs::ReducedBodyState& msg,
							const dwl::ReducedBodyState& state);

		/**
		 * @brief Writes a dwl_msgs::ReducedBodyTrajectory from a
		 * dwl::ReducedBodyTrajectory
		 * @param[out] msg Reduced-body trajectory ros message
		 * @param[in] traj Reduced-body trajectory
		 */
		void writeToMessage(dwl_msgs::ReducedBodyTrajectory& msg,
							const dwl::ReducedBodyTrajectory& traj);

		/**
		 * @brief Writes a dwl::ReducedBodyState from a dwl_msgs::ReducedBodyState
		 * @param[out] state Reduced-body state ros message
		 * @param[in] msg Reduced-body state
		 */
		void writeFromMessage(dwl::ReducedBodyState& state,
							  const dwl_msgs::ReducedBodyState& msg);

		/**
		 * @brief Writes a dwl::ReducedBodyTrajectory from a
		 * dwl_msgs::ReducedBodyTrajectory
		 * @param[out] traj Reduced-body trajectory ros message
		 * @param[in] msg Reduced-body trajectory
		 */
		void writeFromMessage(dwl::ReducedBodyTrajectory& traj,
							  const dwl_msgs::ReducedBodyTrajectory& msg);


	private:
		/** @brief the floating-base system information */
		std::shared_ptr<dwl::model::FloatingBaseSystem> fbs_;

		/** @brief Indicates if it was defined the floating-base system */
		bool is_system_;
};

} //@namespace dwl_msgs

#endif
