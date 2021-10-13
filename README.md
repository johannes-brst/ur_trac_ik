This package provides examples programs to use the standalone TRAC-IK solver and related code.

Currently, there only exists an ik\_tests program that compares KDL's Pseudoinverse Jacobian IK solver with TRAC-IK.  The pr2_arm.launch files runs this test on the default PR2 robot's 7-DOF right arm chain.

###As of v1.4.3, this package is part of the ROS Indigo/Jade binaries: `sudo apt-get install ros-jade-trac-ik`

## build and run

compile with: catkin_make
run with: roslaunch trac_ik_examples pr2_arm.launch

## Requirements