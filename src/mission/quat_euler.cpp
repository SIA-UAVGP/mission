/**
* @file     : mission.h
* @brief    : state machine class
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/08/29
*/

#include <mission/quat_euler.h>

namespace mission
{
	namespace tf
	{

		  /**
			* @brief Convert euler angles to quaternion.
			*/
			Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
			{
				return Eigen::Quaterniond(
						Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
						);
			}
			
			Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q)
			{
				// YPR - ZYX
				return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
			}

			double quaternion_get_yaw(const Eigen::Quaterniond &q)
			{
				// to match equation from:
				// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
				const double &q0 = q.w();
				const double &q1 = q.x();
				const double &q2 = q.y();
				const double &q3 = q.z();

				return std::atan2(2. * (q0*q3 + q1*q2), 1. - 2. * (q2*q2 + q3*q3));
			}
	}	// end of namespace
} // end of namespace