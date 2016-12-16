#include <tool_point_calibration/tool_point_calibration.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "console_tool_calibration");
  ros::NodeHandle pnh ("~");

  // Load user parmameters
  std::string base_frame, tool0_frame;
  int num_samples;

  pnh.param<std::string>("base_frame", base_frame, "base_link");
  pnh.param<std::string>("tool0_frame", tool0_frame, "tool0");
  pnh.param<int>("num_samples", num_samples, 4);

  ROS_INFO("Starting tool calibration with base frame: '%s' and tool0 frame: '%s'.",
           base_frame.c_str(), tool0_frame.c_str());
  ROS_INFO("Move the robot to '%d' different poses, each of which should touch"
           " the tool to the same position in space.\n", num_samples);

  // Create a transform listener to query tool frames
  tf::TransformListener listener;

  // Create storage for user observations
  tool_point_calibration::Affine3dVector observations;
  observations.reserve(num_samples);

  std::string line;
  int count = 0;

  // While ros is ok and there are more counts to be done...
  while (ros::ok() && count < num_samples)
  {
    ROS_INFO("Pose %d: Jog robot to a new location touching the shared position and"
             " press enter.", count);

    std::getline(std::cin, line);

    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(base_frame, tool0_frame, 
                               ros::Time(0), transform);

      Eigen::Affine3d eigen_pose;
      tf::poseTFToEigen(transform, eigen_pose);

      observations.push_back(eigen_pose);

      ROS_INFO_STREAM("Pose " << count << ": captured transform:\n" << eigen_pose.matrix());
      count++;
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  ROS_INFO("Calibration captured %d tool poses (out of %d requested). Computing calibration...",
           count, num_samples);

  tool_point_calibration::TcpCalibrationResult result =
      tool_point_calibration::calibrateTcp(observations);

  ROS_INFO_STREAM("Calibrated tcp (meters in xyz): [" << result.tcp_offset.transpose() << "] from " << tool0_frame);
  ROS_INFO_STREAM("Touch point (meters in xyz): [" << result.touch_point.transpose() << "] in frame " << base_frame);
  ROS_INFO_STREAM("Average residual: " << result.average_residual);
  ROS_INFO_STREAM("Converged: " << result.converged);

  return 0;
}
