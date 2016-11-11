#ifndef TOOL_POINT_CALIBRATION_H
#define TOOL_POINT_CALIBRATION_H

#include <Eigen/Dense>

namespace tool_point_calibration
{

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Affine3dVector;

struct TcpCalibrationResult
{
    /**
     * @brief tcp_offset
     */
    Eigen::Vector3d tcp_offset;

    /**
     * @brief touch_point
     */
    Eigen::Vector3d touch_point;

    /**
     * @brief average_residual
     */
    double average_residual;

    /**
     * @brief converged
     */
    bool converged;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief calibrateTcp
 * @param tool_poses
 * @param tcp_guess
 * @param touch_pt_guess
 * @return
 */
TcpCalibrationResult calibrateTcp(const Affine3dVector& tool_poses,
                                  const Eigen::Vector3d& tcp_guess = Eigen::Vector3d::Zero(),
                                  const Eigen::Vector3d& touch_pt_guess = Eigen::Vector3d::Zero());

}

#endif // TOOL_POINT_CALIBRATION_H
