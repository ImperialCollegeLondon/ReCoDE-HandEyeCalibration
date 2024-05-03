# Background

## Hand-eye calibration
Hand-eye calibration is a well-studied research topic in the field of robotics. It aims to find the transformation matrix from the camera frame to the base frame of the robot. It is an essential procedure for applications that involve the use of cameras because it helps align the camera frame with the robot frame, such that the robot can be commanded to move to a specific position indicated by the camera. Typically, hand-eye calibration problem is treated as an AX=XB/AX=YB problem, where X represents the unknown transformation matrix, and A,B matrices are related to the poses of the robot and camera. Depending on application scenarios, hand-eye calibration algorithms can be divided into marker-based and marker-free approaches, where markers provide easy access to obtaining camera poses. Although marker-based approaches provide higher accuracy, they are usually not suitable to be used for a clinical setup, and hence are constrained to a lab setup. Alternatively, hand-eye calibration can also be treated as a registration problem, where the transformation matrix is found by registering one set of points expressed in one coordinate frame to the same set of points expressed in another frame. In this project, we address hand-eye calibration problem as per the latter school of thought.

## Point set registration through Singular Value Decomposition (SVD)
When correspondences are known between two point sets, the transformation matrix can be analytically calculated using the algorithm proposed in [[1]](#1). A brief summary of the algorithm is listed below. This [link](https://www.youtube.com/watch?v=dhzLQfDBx2Q) provides a video tutorial explaining this algorithm.

## Robot forward kinematics

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
K. S. Arun, T. S. Huang and S. D. Blostein, "Least-Squares Fitting of Two 3-D Point Sets," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. PAMI-9, no. 5, pp. 698-700, Sept. 1987, doi: 10.1109/TPAMI.1987.4767965.

