<!-- An overview of main.cc, an implementation of hand-eye calibration for the da Vinci Research Kit (dVRK) using a bespoke marker -->

# Code overview

<!-- Provide a short description to your project -->

## Files
The `src` directory comprises one `main.cc` and two other *.cc files which contain essential classes and utility functions. The `include` folder contains their corresponding header files (`*.h`). 

## Dependencies
Eigen library, OpenCV, and Point Cloud library are used in this project.

## Workflow
The workflow of `main.cc` is shown in Fig 1, which consists of four steps. Descriptions for each step and their code implementations are presented below.

| ![workflow](../Pics_for_demo/Flowchart.jpg "workflow") |
|:--:|
| Fig 1. Workflow |

## Step 1: Read input parameters
### 1. Mechanical parameters
Mechanical parameters include DH parameters of the tool instrument, radii of two spherical markers, and their relative position when placed on the tool shaft. When reading input parameters, function `ReadParameter` is used. The corresponding code block is presented below

```cpp
    ReadParameter(ParameterPath, "ball_1_radius", ball_1_radius);
    ReadParameter(ParameterPath, "ball_2_radius", ball_2_radius);
    ReadParameter(ParameterPath, "l1", l1);
    ReadParameter(ParameterPath, "l2", l2);
```

### 2. Camera parameters
Camera parameters include the intrinsic matrix of the RGB lens and the extrinsic matrix between the RGB and depth lens. They are both $\textrm{4} \times \textrm{4}$ matrices with numbers of double precision, and hence we use the function `ReadMatrixFromTxt` to read data

```cpp
    ReadMatrixFromTxt(Intrinsic_path, Intrinsic_matrix);
    ReadMatrixFromTxt(Extrinsic_path, Extrinsic_matrix);
```

### 3. Vision data
Vision data includes recorded infrared images, colour images and point clouds captured by the depth camera as the instrument was being moved to different positions. We first delare a class object to process vision data. 

```cpp
    FeatureDetection DepthCamera;
```

Then we feed camera parameters into this object

```cpp
    DepthCamera.ReadIntrinsicMatrix(Intrinsic_matrix);
    DepthCamera.ReadAcusenseDepth2RGBMat(Extrinsic_matrix);
```

When reading image frames. we simply call back the built-in OpenCV function `cv::imread`, and when we read point cloud input, we use the class function `FeatureDetection::ReadPointCloudPCD`.

### 4. Kinematics data
Kinematics data in this project only contains recorded joint angle positions when the instrument was being moved to different positions. These recorded positions are stored in an Eigen matrix, and hence to read these inputs we still call back the function `ReadMatrixFromTxt`. Then we convert joint angle positions to joint 4 position in the robot base frame via forward kinematics using the function `GetJoint4Position`

```cpp
    GetJoint4Position(JointAngleList, Joint4PosList_robot, l1, l2);
```

## Step 2: Find out positions in the robot frame


## Step 3: Find out positions in the camera frame

## Step 4: Registration with given correspondences

## Output files


