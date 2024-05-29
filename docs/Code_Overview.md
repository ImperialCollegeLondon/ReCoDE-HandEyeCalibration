<!-- An overview of main.cc, an implementation of hand-eye calibration for the da Vinci Research Kit (dVRK) using a bespoke marker -->

# Code overview

<!-- Provide a short description to your project -->

## Files
The `src` directory comprises one `main.cc` and two other *.cc files which contain essential classes and utility functions. The `include` folder contains their corresponding header files (`*.h`). 

## Dependencies
Eigen library, OpenCV, and Point Cloud library are used in this project.

## Step 1: Reading Input, class initialisation
- 
During data collection, the robot was moved to `n_frames` different positions, and at each position, the camera captured the pose of the markers. The collected historical joint angles are read in as `JointAngleList`, and joint 4 positions can be then calculated using forward kinematics and stored as `Joint4PosList_robot`.
```
    std::string JointAnglePath = GetCurrentWorkingDir() + "/../q_history.txt";
    Eigen::MatrixXd JointAngleList, Joint4PosList_robot;
    ReadMatrixFromTxt(JointAnglePath, JointAngleList);
    GetJoint4Position(JointAngleList, Joint4PosList_robot); // from Denavit–Hartenberg matrix
```
```
    std::string Intrinsic_path = GetCurrentWorkingDir() + "/../Acusense_RGB_K.txt", 
                Extrinsic_path = GetCurrentWorkingDir() + "/../ExtrinsicMat.txt",
                RGB_folder_path = GetCurrentWorkingDir() + "/../RGB/",
                IR_folder_path = GetCurrentWorkingDir() + "/../IR/",
                PCD_folder_path = GetCurrentWorkingDir() + "/../pcd/";
    Eigen::MatrixXd Intrinsic_matrix, Extrinsic_matrix;
    ReadMatrixFromTxt(Intrinsic_path, Intrinsic_matrix);
    ReadMatrixFromTxt(Extrinsic_path, Extrinsic_matrix);

    FeatureDetection DepthCamera;
    DepthCamera.ReadIntrinsicMatrix(Intrinsic_matrix);
    DepthCamera.ReadAcusenseDepth2RGBMat(Extrinsic_matrix);
```
## Step 2: Find out positions in the robot frame

## Step 3: Find out positions in the camera frame

## Step 4: Registration with given correspondences

## Output files


