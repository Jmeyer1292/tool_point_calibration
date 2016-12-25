# Tool Point Calibration
Provides a library and simple command line utility for doing 3 DOF (X, Y, Z) calibration of a robot's tool through the "touch point" method wherein the user jogs the robot to several poses where the tool to be calibrated touches the same point from different orientations.

## Installation
The core tcp calibration library only requires the non-linear optimization solver Ceres to build. See [this link](http://ceres-solver.org/installation.html) for details on installation.

The console calibration utility uses ROS' TF libraries.

## Use
To use:
  1. Start up your robots driver as normal. A node like `robot_state_publisher` should be publishing TF frames of your robot.

  2. Identify a base frame, either the base frame of the robot or the origin of the scene. This will be used for TF lookups, and the `touch point` will be reported in this frame.

  3. Identify a tool0 frame. This is the frame from which you wish to calibrate a TCP. This will typically be the flange of the robot `tool0`.

  4. Run the calibration node:
  ```
  roslaunch tool_point_calibration console_calibration.launch base_frame:=YOUR_BASE_FRAME tool0_frame:=YOUR_TOOL0_FRAME num_samples:=NUMBER_OF_TOUCH_POINTS
  ```
  By default base frame is `base_frame`, tool0_frame is `tool0`, and num_samples is `4`. I recommend you use at least 4
  samples.

  5. Jog the robot so that the tool you wish to calibrate is touching a known point in the world, such as the tip of a nail or the corner of a work-piece. When your're happy, press enter in the console window from which you ran the tcp calibration routine. You should see a 4x4 matrix print out.

  6. Repeat step 5 at least three more times. Each time approach the fixed point in the world from a different orientation. If you wish to abort early, control-C the node.

  7. When you've captured all of the required points, the calibration will be automatically run and the output displayed to the screen.