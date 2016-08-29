/**
* @file     : mission.h
* @brief    : state machine class
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/08/29
*/

#pragma once 

#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace mission
{
	namespace tf
	{

		/**
		 * @brief Convert euler angles to quaternion.
		 */
		Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);

		/**
		* @brief Convert euler angles to quaternion.
		*/
		inline Eigen::Quaterniond quaternion_from_rpy(const double roll, const double pitch, const double yaw) {
			return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
		}

		/**
		* @brief Convert quaternion to euler angles
		*
		* Reverse operation to @a quaternion_from_rpy()
		*/
		Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q);

		/**
		* @brief Get Yaw angle from quaternion
		*/
		double quaternion_get_yaw(const Eigen::Quaterniond &q);


	}	// end of namespace
} // end of namespace