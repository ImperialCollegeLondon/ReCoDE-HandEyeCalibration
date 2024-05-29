<!-- An overview of main.cc, an implementation of hand-eye calibration for the da Vinci Research Kit (dVRK) using a bespoke marker -->

# Code overview

<!-- Provide a short description to your project -->

## Files
The `src` directory comprises one `main.cc` and two other *.cc files which contain essential classes and utility functions. The `include` folder contains their corresponding header files (`*.h`). 

## Dependencies
Eigen library, OpenCV, and Point Cloud library are used in this project.

## Step 1: Reading Input, class initialisation
During data collection, the robot was moved to `n_frames` different positions, and at each position, the camera captured the pose of the markers. The collected historical joint angles are read in as `JointAngleList`, and joint 4 positions can be then calculated using forward kinematics and stored as `Joint4PosList_robot`.
```
    std::string JointAnglePath = GetCurrentWorkingDir() + "/../q_history.txt";
    Eigen::MatrixXd JointAngleList, Joint4PosList_robot;
    ReadMatrixFromTxt(JointAnglePath, JointAngleList);
    GetJoint4Position(JointAngleList, Joint4PosList_robot); // from Denavit–Hartenberg matrix
```
Input image data include the collected colour frames, infrared frames and point clouds. The intrinsic and extrinsic parameters of the camera are stored as `Intrinsic_matrix` and `Extrinsic_matrix`, respectively.
```
    std::string Intrinsic_path = GetCurrentWorkingDir() + "/../Acusense_RGB_K.txt", 
                Extrinsic_path = GetCurrentWorkingDir() + "/../ExtrinsicMat.txt",
                RGB_folder_path = GetCurrentWorkingDir() + "/../RGB/",
                IR_folder_path = GetCurrentWorkingDir() + "/../IR/",
                PCD_folder_path = GetCurrentWorkingDir() + "/../pcd/";
    Eigen::MatrixXd Intrinsic_matrix, Extrinsic_matrix;
    ReadMatrixFromTxt(Intrinsic_path, Intrinsic_matrix);
    ReadMatrixFromTxt(Extrinsic_path, Extrinsic_matrix);
```
After reading these input data, we can then initialise a camera object `DepthCamera` for image processing.
```
    FeatureDetection DepthCamera;
    DepthCamera.ReadIntrinsicMatrix(Intrinsic_matrix);
    DepthCamera.ReadAcusenseDepth2RGBMat(Extrinsic_matrix);
```
## Step 2: Find out positions in the robot frame
As mentioned in step 1, joint 4 positions in the camera frame are calculated using forward kinematics
```
GetJoint4Position(JointAngleList, Joint4PosList_robot); // from Denavit–Hartenberg matrix
```

## Step 3: Find out positions in the camera frame
We iterate through all the collected image frames, calulate joint 4 positions in the camera frame. For each robot pose, we read in its corresponding infrared image and point cloud to construct a depth mask.
```
    // Read PCD file
    PointCloudT::Ptr frame_cloud(new PointCloudT);
    cv::Mat img_depth, img_IR;
    std::string pcd_file_path = PCD_folder_path + "frame" + std::to_string(count) + ".pcd",
                IR_file_path = IR_folder_path + "frame" + std::to_string(count) + ".jpg";
    img_IR = cv::imread(IR_file_path); // Red IR frame first, because we want to create a depth frame of the same resolution as IR frame
    DepthCamera.ReadPointCloudPCD(pcd_file_path, img_IR.rows, img_IR.cols, img_depth, frame_cloud);
```
We can leverage the `DepthCamera` object to process the image data and return joint 4 position in the camera frame.
```
    // j4 position reconstruction
    Eigen::Vector3d pos_j4_camera = DepthCamera.ReconstructJ4Position(img_depth, frame_cloud, ball_1_radius, ball_2_radius); // unit (mm)
    pos_j4_camera *= 1e-3; // unit (m)
```
Then we stack them into a matrix `Joint4PosList_camera`
```
    Joint4PosList_camera.row(count) = pos_j4_camera;
```
## Step 4: Registration with given correspondences

## Output files


