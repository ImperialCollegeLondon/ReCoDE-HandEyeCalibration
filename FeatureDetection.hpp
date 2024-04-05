#ifndef FEATURE_DETECTION_HPP
#define FEATURE_DETECTION_HPP
#include "utils.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

class FeatureDetection
{
    public:
        FeatureDetection(){};
        ~FeatureDetection(){}; // deconstructor
        void ReadIntrinsicMatrix(const Eigen::MatrixXd &Intrinsic_mat);
        void ReadPointCloudPCD(const std::string &file_path, const int &n_rows, const int &n_cols, cv::Mat &img, PointCloudT::Ptr &cloud); // Read PLY file and convert it into a BW image and a point cloud
        Eigen::Vector3d ReconstructJ4Position(const cv::Mat &img_depth, const PointCloudT::Ptr &cloud, const double &ball_1_radius, const double &ball_2_radius);
        void ReadHandEyeTransform(const Eigen::Matrix4d &T);
        void ReadAcusenseDepth2RGBMat(const Eigen::Matrix4d &T_d_rgb) {T_depth2RGB_Acusense = T_d_rgb; Eigen::Vector3d t_vec = T_d_rgb.topRightCorner(3,1) * 1e-3; T_depth2RGB_Acusense.topRightCorner(3,1) = t_vec;}; // Unit (m)
        void cvtDepth2Colour_Acusense(const Eigen::Vector3d &pt_depth, Eigen::Vector3d &pt_colour);
        void drawShaftAxisColourAcusense(cv::Mat img, std::string window_name, const Eigen::Vector3d &origin, const Eigen::Vector3d &end_pt);
        void cvt2cameraFrame(const Eigen::Vector3d &point_robot, Eigen::Vector3d &point_camera);
    private:
        Eigen::Matrix3d K_int;
        void ContourDetection(const cv::Mat image, int min_area, std::vector<cv::Mat> &contour_img_list);
        void Fit3DSphere(const std::vector<cv::Point3d> &pt_list, double &xc, double &yc, double &zc, double &rc);
        void FindBallCentres(cv::Mat img, PointCloudT::Ptr cloud, double radius_ball_1, double radius_ball_2, Eigen::Vector3d &centre_ball_1, Eigen::Vector3d &centre_ball_2);
        void world2pixel_Acusense(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D);
        void world2pixel(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D);
        double Lbb = 0.050,
               Lbp = 0.017; // ball1 ~ ball2 (m), ball2 ~ pitch (m), respectively
        double fx, fy, cx, cy;
        Eigen::Matrix4d Tcr, Trc, T_depth2RGB_Acusense;
};

#endif

