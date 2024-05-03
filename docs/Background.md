# Background

## Hand-eye calibration
Hand-eye calibration is a well-studied research topic in the field of robotics. It aims to find the transformation matrix from the camera frame to the base frame of the robot $${\color{red}Pic hand-eye}$$. It is an essential procedure for applications that involve the use of cameras because it helps align the camera frame with the robot frame, such that the robot can be commanded to move to a specific position indicated by the camera. Typically, hand-eye calibration problem is treated as an **AX=XB**[[1]](#1) (**AX=YB**[[2]](#2)) problem, where X represents the unknown transformation matrix, and A,B matrices are related to the poses of the robot and camera. Depending on application scenarios, hand-eye calibration algorithms can be divided into marker-based and marker-free approaches, where markers provide easy access to obtaining camera poses. Although marker-based approaches provide higher accuracy, they are usually not suitable to be used for a clinical setup, and hence are constrained to a lab setup. Alternatively, hand-eye calibration can also be treated as a registration problem, where the transformation matrix is found by registering one set of points expressed in one coordinate frame to the same set of points expressed in another frame. In this project, we address hand-eye calibration problem as per the latter school of thought.

## Point set registration through Singular Value Decomposition (SVD)
When correspondences are known between two point sets, the transformation matrix can be analytically calculated using the algorithm proposed in [[3]](#3). A brief summary of the algorithm is listed below. [link](https://www.youtube.com/watch?v=dhzLQfDBx2Q) provides a video tutorial explaining this algorithm.

**What is singular value decomposition?**

## Robot forward kinematics
In robotics, we describe the status of a robot in either the joint space or Cartesian space $${\color{red}Pic robot frames}$$. The joint space describes robot pose relative to joint coordinate frames, while the Cartesian space describes robot pose relative to the base frame of the robot *O<sub>0</sub>*. Different combinations of joint positions result in different poses of a robot, and this relationship is described by forward kinematics. Forward kinematics depends on both the joint status and the mechanical parameters of a robot, which are called Denavit-Hartenberg (DH) parameters. The selection DH parameters depends on how joint coordinate frames are established and hence is not unique to a robot. The selection of DH parameters can be referred to [[4]](#4). From DH parameters, we can construct a $4\cross4$ matrix **T<sub>i,i+1</sub>** to describe the transformation between two adjacent joint coordinate frames *O<sub>i</sub>* and *O<sub>i+1</sub>*. Therefore, with knowledge of the forward kinematics and the position of point *P<sub>j</sub>* in joint *j* coordinate frame, we can recover its position in the robot base frame *P<sub>0</sub>* as
$$P_0 = \prod_{k=0}^{j-1}T_{k,k+1}P_j$$


## Experimental setup
- The first-generation da Vinci Research Kit (dVRK)
    - DH parameters
    - Instrument (Long Needle Driver)
- Acusense camera 
    - RGB lens
    - Depth lens
- Bespoke markers

## References
<a id="1">[1]</a> 
Shiu, Y.C. and Ahmad, S., 1987. Calibration of wrist-mounted robotic sensors by solving homogeneous transform equations of the form AX= XB.
<a id="2">[2]</a> 
Shah, M., Eastman, R.D. and Hong, T., 2012, March. An overview of robot-sensor calibration methods for evaluation of perception systems. In Proceedings of the Workshop on Performance Metrics for Intelligent Systems (pp. 15-20).
<a id="3">[3]</a> 
Arun, K.S., Huang, T.S. and Blostein, S.D., 1987. Least-squares fitting of two 3-D point sets. IEEE Transactions on pattern analysis and machine intelligence, (5), pp.698-700.
<a id="4">[4]</a> 
[https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)











