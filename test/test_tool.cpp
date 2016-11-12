#include <iostream>
#include <random>

#include <tool_point_calibration/tool_point_calibration.h>
#include <gtest/gtest.h>

using Affine3dVector = std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>;

static std::random_device rd;

static double random_number()
{
    static std::uniform_real_distribution<double> dist (0.0, 1.0);
    return dist(rd);
}

static double random_normal_number()
{
    const static double stddev = 0.005;
    static std::normal_distribution<double> norm_dist(0.0, stddev);
    return norm_dist(rd);
}

Affine3dVector makeFakePoses(const Eigen::Vector3d& center, const Eigen::Affine3d& tool_offset, int n)
{
    Affine3dVector vector;
    auto inverse = tool_offset.inverse();

    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector3d norm (random_number(), random_number(), random_number());
        norm.normalize();

        Eigen::Affine3d delta = ( Eigen::AngleAxisd(3.14159, norm) * inverse );

        Eigen::Affine3d toolpt = Eigen::Translation3d(center) * delta * Eigen::Translation3d(random_normal_number(),
                                                                                             random_normal_number(),
                                                                                             random_normal_number());
        vector.push_back(toolpt);

        // std::cout << "TOOL:\n" << toolpt.matrix() << "\n";
    }

    return vector;
}


TEST(tcp_calibration, initial_tests)
{
    Eigen::Affine3d tool_def;
    tool_def = Eigen::Translation3d(0.5, 0.25, 0) *
            Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitX());

    std::cout << "TOOL POSE\n" << tool_def.matrix() << "\n\n";

    Eigen::Vector3d center (1,1,2);
    auto points = makeFakePoses(center, tool_def, 100);

//    for (const auto& pose : points)
//    {
//      Eigen::Affine3d t;
//      t = pose * tool_def;
//      std::cout << t.translation().matrix().transpose() << "\n";
//    }

    tool_point_calibration::TcpCalibrationResult result =
            tool_point_calibration::calibrateTcp(points);

    std::cout << "TCP: " << result.tcp_offset.transpose() << "\n";
    std::cout << "Touch point: " << result.touch_point.transpose() << "\n";
    std::cout << "Avg Residual: " << result.average_residual << "\n";
    std::cout << "Converged?: " << result.converged << "\n";

    double delta = (result.tcp_offset - tool_def.translation()).norm();

    EXPECT_TRUE(std::abs(delta) < 0.005);
}
