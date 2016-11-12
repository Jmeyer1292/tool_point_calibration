#include <tool_point_calibration/tool_point_calibration.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "console_tool_calibration");
  ros::NodeHandle pnh ("~");

  std::string base_frame, tool0_frame;

  pnh.param<std::string>("base_frame", base_frame, "base_frame");
  pnh.param<std::string>("tool0_frame", tool0_frame, "tool0");

  tf::TransformListener listener;

  tool_point_calibration::Affine3dVector observations;

  ros::AsyncSpinner spinner (1); // required for listener to operate in background?
  std::string line;
  int count = 0;
  while (pnh.ok() && count < 5)
  {
    std::getline(std::cin, line);

    ROS_INFO("Attempting to capture robot tool frame pose");


    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(base_frame, tool0_frame, 
                               ros::Time(0), transform);

      Eigen::Affine3d eigen_pose;
      tf::poseTFToEigen(transform, eigen_pose);

      observations.push_back(eigen_pose);

      ROS_INFO_STREAM("Captured robot pose:\n" << eigen_pose.matrix());

      count++;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

  }

    ROS_INFO("Performing calibration");
    tool_point_calibration::TcpCalibrationResult result =
        tool_point_calibration::calibrateTcp(observations);

    std::cout << "TCP: " << result.tcp_offset.transpose() << "\n";
    std::cout << "Touch point: " << result.touch_point.transpose() << "\n";
    std::cout << "Avg Residual: " << result.average_residual << "\n";
    std::cout << "Converged?: " << result.converged << "\n";

  return 0;
}