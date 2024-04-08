#ifndef FEATURE_DETECTION_HPP
#define FEATURE_DETECTION_HPP
#include "utils.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

/**
 * @brief A class that contains computer vision methods that are used in the project
 * 
 */
class FeatureDetection
{
    public:
        /**
         * @brief Construct a new Feature Detection object
         * 
         */
        FeatureDetection(){};
        /**
         * @brief Destroy the Feature Detection object
         * 
         */
        ~FeatureDetection(){}; 
        /**
         * @brief Read in the intrinsic matrix of the RGB lens
         * 
         * @param Intrinsic_mat 3*3 camera intrinsic matrix
         */
        void ReadIntrinsicMatrix(const Eigen::MatrixXd &Intrinsic_mat);
        /**
         * @brief Read in pcd file and save it as a black and white image, where white pixels correspond to points that have depth values
         * 
         * @param file_path path to the pcd file
         * @param n_rows number of rows of the image
         * @param n_cols number of columns of the image
         * @param img output image
         * @param cloud output point cloud pointer
         */
        void ReadPointCloudPCD(const std::string &file_path, const int &n_rows, const int &n_cols, cv::Mat &img, PointCloudT::Ptr &cloud);
        /**
         * @brief a global function for finding joint 4 position in the camera frame
         * 
         * @param img_depth black and image where white only pixels have depth values (unit (mm))
         * @param cloud input point cloud pointer
         * @param ball_1_radius radius of the larger spherical marker, unit (mm)
         * @param ball_2_radius radius of the smaller spherical marker, unit (mm)
         * @return Eigen::Vector3d output reconstructed 3d position of joint 4 in the camera frame, unit (mm)
         */
        Eigen::Vector3d ReconstructJ4Position(const cv::Mat &img_depth, const PointCloudT::Ptr &cloud, const double &ball_1_radius, const double &ball_2_radius);
        /**
         * @brief Read 4*4 homogeneous hand-eye transformation matrix
         * 
         * @param T 4*4 homogeneous transformation matrix
         */
        void ReadHandEyeTransform(const Eigen::Matrix4d &T);
        /**
         * @brief Read extrinsic matrix that transforms from the RGB frame to the depth frame
         *  
         * @param T_d_rgb extrinsic matrix
         */
        void ReadAcusenseDepth2RGBMat(const Eigen::Matrix4d &T_d_rgb) {T_depth2RGB_Acusense = T_d_rgb; Eigen::Vector3d t_vec = T_d_rgb.topRightCorner(3,1) * 1e-3; T_depth2RGB_Acusense.topRightCorner(3,1) = t_vec;}; // Unit (m)
        /**
         * @brief convert a 3D point in the depth frame to the RGB frame
         * 
         * @param pt_depth input position of a 3D point in the depth frame
         * @param pt_colour output position of the 3D point in the RGB frame
         */
        void cvtDepth2Colour_Acusense(const Eigen::Vector3d &pt_depth, Eigen::Vector3d &pt_colour);
        /**
         * @brief Overlay the tool shaft axis on the colour image
         * 
         * @param img Colour image
         * @param window_name name of the window
         * @param origin 3D position of the starting point on the tool shaft, expressed in the depth frame
         * @param end_pt 3D position of the ending point on the tool shaft, expressed in the depth frame
         */
        void drawShaftAxisColourAcusense(cv::Mat img, std::string window_name, const Eigen::Vector3d &origin, const Eigen::Vector3d &end_pt);
        /**
         * @brief convert a 3D point the robot base frame to the depth frame
         * 
         * @param point_robot input position of a 3D point in the robot base frame
         * @param point_camera output position of the 3D point in the depth frame
         */
        void cvt2cameraFrame(const Eigen::Vector3d &point_robot, Eigen::Vector3d &point_camera);
    private:
        /**
         * @brief 3*3 intrinsic matrix of the RGB lens
         * 
         */
        Eigen::Matrix3d K_int;
        /**
         * @brief Detect contours in the black and white image
         * 
         * @param image input black and white image
         * @param min_area minimum area of the contour to be detected
         * @param contour_img_list vector of output images of infilled detected contours.
         */
        void ContourDetection(const cv::Mat image, int min_area, std::vector<cv::Mat> &contour_img_list);
        /**
         * @brief Find out the 3D position of the centre of a spherical marker
         * 
         * @param pt_list vector of 3d points on the surface of the marker
         * @param xc output x coordinate of the centre of the marker, unit (mm)
         * @param yc output y coordinate of the centre of the marker, unit (mm)
         * @param zc output z coordinate of the centre of the marker, unit (mm)
         * @param rc output radius of the marker, unit (mm)
         */
        void Fit3DSphere(const std::vector<cv::Point3d> &pt_list, double &xc, double &yc, double &zc, double &rc);
        /**
         * @brief Find out the 3D positions of the centre of two spherical markers
         * 
         * @param img input black and white image
         * @param cloud input point cloud
         * @param radius_ball_1 radius of the larger marker, unit (mm)
         * @param radius_ball_2 radius of the smaller marker, unit (mm)
         * @param centre_ball_1 output 3D position of the centre of the larger marker in the depth frame, unit (mm)
         * @param centre_ball_2 output 3D position of the centre of the smaller marker in the depth frame, unit (mm)
         */
        void FindBallCentres(cv::Mat img, PointCloudT::Ptr cloud, double radius_ball_1, double radius_ball_2, Eigen::Vector3d &centre_ball_1, Eigen::Vector3d &centre_ball_2);
        /**
         * @brief project a 3D point in the depth frame to the RGB image plane
         * 
         * @param point_3D input 3D position of the point
         * @param pixel_2D output pixel coordinates of the back-projected 3D point in the RGB image plane
         */
        void world2pixel_Acusense(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D);
        /**
         * @brief project a 3D point in the depth frame to the depth image plane
         * 
         * @param point_3D input 3D position of the point
         * @param pixel_2D output pixel coordinates of the back-projected 3D point in the depth image plane
         */
        void world2pixel(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D);
        /**
         * @brief parameters of the designed markers
         * 
         */
        double Lbb = 0.050, Lbp = 0.017; // ball1 ~ ball2 (m), ball2 ~ pitch (m), respectively
        /**
         * @brief Intrinsic parameters
         * 
         */
        double fx, fy, cx, cy;
        Eigen::Matrix4d Tcr, Trc, T_depth2RGB_Acusense;
};

#endif

