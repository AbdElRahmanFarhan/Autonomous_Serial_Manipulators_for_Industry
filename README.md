# Autonomous_Serial_Manipulators_for_Insdustry
Thank you for visiting, we appreciate your time!
This repository is for a ping playing robot, we used an ABB IRB120 robot provided by our university (Ain shams university), we used Intel RealSense D435.

This is a premature version of the ABB ping-pong playing robot repository, In the current stage the robot is able to rebuff a ping pong ball passing through its workspace, we are still developing and updating the packages to be used easily by your side.

In this repository, you'll find two main packages, first one is the vision system for a ping-pong playing robot, which contains the ball detection, tracking, and prediction algorithms.
Second package is responnsible for the robot control in our project, which includes a safe time optimal path parameterization algorithm, a smart hitting point decision algorithm, robot-controller interface which is divided into a low level controller (EGM) and high level controller(RWS).

### Dependencies
- Moveit!
- RWS & EGM abb controller interfaces
- Librealsense SDK 2.0
- ROS
- [STOPP](https://github.com/AbdelrhmanBassiouny/stopp) (Developed as part of this project, and is now publicly available through pip)

Feel free to contact us for any inquiries!

### Demonstration

https://user-images.githubusercontent.com/36744004/198647843-711be5be-7433-4769-84ab-d28c291e5ad3.mp4

