#include "utils.hpp"
#include "FeatureDetection.hpp"

// Define a function to return the current working directory
#include <stdio.h>  /* defines FILENAME_MAX */
// #define WINDOWS  /* uncomment this line to use it for windows.*/ 
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d);

std::string GetCurrentWorkingDir(void) 
{
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

int n_frames = 50; // Altogether there were 50 frames collected

int main()
{
    /**
     * @brief Step 1. read joint angles and calculate j4 positions in the robot frame
     * 
     */
    std::string JointAnglePath = GetCurrentWorkingDir() + "/../q_history.txt";
    Eigen::MatrixXd JointAngleList, Joint4PosList_robot;
    ReadMatrixFromTxt(JointAnglePath, JointAngleList);
    GetJoint4Position(JointAngleList, Joint4PosList_robot); // from Denavit–Hartenberg matrix
    /**
     * @brief Step 2, recover j4 positions in the camera frame using computer vision techniques
     * 
     */
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

    // Iterate over each collected frame
    int count = 0;
    double ball_1_radius = 12.0, ball_2_radius = 10.0; // unit (mm)
    Eigen::MatrixXd Joint4PosList_camera(n_frames, 3); // unit (m)
    for(count; count < n_frames; count++)
    {
      // Read PCD file
      PointCloudT::Ptr frame_cloud(new PointCloudT);
      cv::Mat img_depth, img_IR;
      std::string pcd_file_path = PCD_folder_path + "frame" + std::to_string(count) + ".pcd",
                  IR_file_path = IR_folder_path + "frame" + std::to_string(count) + ".jpg";
      img_IR = cv::imread(IR_file_path); // Red IR frame first, because we want to create a depth frame of the same resolution as IR frame
      DepthCamera.ReadPointCloudPCD(pcd_file_path, img_IR.rows, img_IR.cols, img_depth, frame_cloud);
      // j4 position reconstruction
      Eigen::Vector3d pos_j4_camera = DepthCamera.ReconstructJ4Position(img_depth, frame_cloud, ball_1_radius, ball_2_radius); // unit (mm)
      pos_j4_camera *= 1e-3; // unit (m)
      Joint4PosList_camera.row(count) = pos_j4_camera;
    }
    Eigen::Matrix4d Trc = SVD_rigid_transform(Joint4PosList_camera, Joint4PosList_robot),
                    Tcr = Trc.inverse();

    // Overlay tool axis
    DepthCamera.ReadHandEyeTransform(Tcr);
    Eigen::Vector3d rcm_pos_robot = Eigen::Vector3d::Zero(),
                    j4_pos_robot,
                    rcm_pos_cam, j4_pos_cam;
    j4_pos_robot << Joint4PosList_robot(49,0), Joint4PosList_robot(49,1), Joint4PosList_robot(49,2);
    cv::Mat img_colour = cv::imread(RGB_folder_path + "frame49.jpg");
    DepthCamera.cvt2cameraFrame(rcm_pos_robot, rcm_pos_cam);
    DepthCamera.cvt2cameraFrame(j4_pos_robot, j4_pos_cam);
    DepthCamera.drawShaftAxisColourAcusense(img_colour, "overlay", rcm_pos_cam, j4_pos_cam);
    // SVD to find out Hand-eye transformation matrix
    std::cout<<Trc<<std::endl;
    return 1.0;
}