#include "FeatureDetection.hpp"

void FeatureDetection::ReadIntrinsicMatrix(const Eigen::MatrixXd &Intrinsic_mat)
{
    this->K_int = Intrinsic_mat;
    fx = K_int(0,0);
    fy = K_int(1,1);
    cx = K_int(0,2);
    cy = K_int(1,2);
}

void FeatureDetection::ReadHandEyeTransform(const Eigen::Matrix4d &T)
{
    Tcr.setZero(); 
    Tcr = T; 
    Trc = Tcr.inverse();
}

void FeatureDetection::cvtDepth2Colour_Acusense(const Eigen::Vector3d &pt_depth, Eigen::Vector3d &pt_colour)
{
    Eigen::Matrix3d rot = T_depth2RGB_Acusense.topLeftCorner(3,3);
    Eigen::Vector3d trans = T_depth2RGB_Acusense.topRightCorner(3,1);
    Eigen::Vector3d pt_tmp = pt_depth + trans;
    pt_colour = rot * pt_tmp;
}

void FeatureDetection::world2pixel(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D)
{
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
    double x1 = point_3D[0] / point_3D[2],
           y1 = point_3D[1] / point_3D[2],
           r = x1*x1 + y1*y1,
           x2 = x1 * (1+k1*pow(r,2)+k2*pow(r,4)+k3*pow(r,6)) + 2*p1*x1*y1 + p2*(r*r + 2*x1*x1),
           y2 = y1 * (1+k1*pow(r,2)+k2*pow(r,4)+k3*pow(r,6)) + p1*(pow(r,2)+2*pow(y1,2)) + 2*p2*x1*y1;
    double x = fx * x2 + cx,
           y = fy * y2 + cy;
    pixel_2D.setZero();
    pixel_2D << x, y;
}

void FeatureDetection::world2pixel_Acusense(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D)
{
    Eigen::Matrix3d rot = T_depth2RGB_Acusense.topLeftCorner(3,3);
    Eigen::Vector3d trans = T_depth2RGB_Acusense.topRightCorner(3,1);
    Eigen::Vector3d pt_tmp = point_3D + trans,
                    pt_rgb = rot * pt_tmp;
    world2pixel(pt_rgb, pixel_2D);
}

void FeatureDetection::drawShaftAxisColourAcusense(cv::Mat img, std::string window_name, const Eigen::Vector3d &origin, const Eigen::Vector3d &end_pt)
{
    Eigen::Vector3d axis_end = end_pt;
    Eigen::Vector2d origin_2d, end_2d;
    world2pixel_Acusense(origin, origin_2d);
    world2pixel_Acusense(axis_end, end_2d);
    
    int origin_u = int(origin_2d[0]), origin_v = int(origin_2d[1]),
        end_u = int(end_2d[0]), end_v = int(end_2d[1]);
    cv::Point origin_cv (origin_u, origin_v), end_cv (end_u, end_v);
    cv::Scalar green(0,255,0), red(0,0,255);
    cv::Mat img_copy = img.clone();
    DrawDashedLine(img_copy, origin_cv, end_cv, green, 2, "dotted", 10);
    cv::circle(img_copy, end_cv, 3, red, -1);
    cv::imshow(window_name, img_copy);
    cv::waitKey(0);
}

void FeatureDetection::cvt2cameraFrame(const Eigen::Vector3d &point_robot, Eigen::Vector3d &point_camera)
{
    auto rot = Tcr.topLeftCorner(3,3), translate = Tcr.topRightCorner(3,1);
    point_camera = rot * point_robot + translate;
}

void FeatureDetection::ReadPointCloudPCD(const std::string &file_path, const int &n_rows, const int &n_cols, cv::Mat &img, PointCloudT::Ptr &cloud)
{
    // Load pcd first
    cloud->clear();  
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    // Construct depth image
    img = cv::Mat(n_rows, n_cols, CV_8UC1, cv::Scalar(0)); // first create a purely black image
    for(size_t i=0; i < n_rows; i++)
    {
        for(size_t j=0; j< n_cols; j++)
        {
            int index = i * n_cols + j;
            double pixel_depth = cloud->points[index].z;
            if(pixel_depth != 0.0)
            {
                img.at<uchar>(i,j) = 255;
            }
        }
    }
    // cv::imshow("depth_img", img);
    // cv::waitKey(0.0);
}

void FeatureDetection::ContourDetection(const cv::Mat image, int min_area, std::vector<cv::Mat> &contour_img_list)
{
    contour_img_list.clear();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(),[](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2)
              {return cv::contourArea(c1,false) > cv::contourArea(c2, false);}); // sort based on a descending order in contour areas
    for(int i=0; i<contours.size(); i++)
    { 
        if(cv::contourArea(contours[i]) > min_area)
        {
            auto area = cv::contourArea(contours[i]);
            // std::cout<<"contour area = "<<area<<std::endl;
            cv::Mat image_with_contour = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
            cv::drawContours(image_with_contour, contours, i, cv::Scalar(255), -1, 8,hierarchy);
            // std::string window_name = "find contours" + std::to_string(i);
            // cv::imshow(window_name, image_with_contour);
            // cv::waitKey(0.0);
            contour_img_list.push_back(image_with_contour);
        }
    }
}

void FeatureDetection::Fit3DSphere(const std::vector<cv::Point3d> &pt_list, double &xc, double &yc, double &zc, double &rc)
{
    int n_pts = pt_list.size();
    Eigen::MatrixXd A(n_pts, 4), b(n_pts,1);
    for(size_t i=0; i<n_pts; i++)
    {
        double x = pt_list[i].x, y = pt_list[i].y, z = pt_list[i].z;
        A(i,0) = x; A(i,1) = y; A(i,2) = z; A(i,3) = 1;
        b(i,0) = x*x + y*y + z*z;
    }
    Eigen::VectorXd c = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    xc = c[0]/2; yc = c[1]/2; zc = c[2]/2;
    rc = sqrt(c[3]+xc*xc+yc*yc+zc*zc);
}

void FeatureDetection::FindBallCentres(cv::Mat img, PointCloudT::Ptr cloud, double radius_ball_1, double radius_ball_2, Eigen::Vector3d &centre_ball_1, Eigen::Vector3d &centre_ball_2)
{
    // Step 1, find contours
    std::vector<cv::Mat> contour_img_list;
    std::vector<double> radius_list = {radius_ball_1, radius_ball_2}; // unit (mm), r1 > r2
    std::vector<cv::Point3d> ball_centre_list(2);
    int min_area = 400;
    this->ContourDetection(img, min_area, contour_img_list);
    assert(contour_img_list.size() >= 2 && "Not enough ball contours detected");
    for(size_t i=0; i<2; i++)
    {
        cv::Mat contour_img = contour_img_list[i];
        // std::string window_name = "contour" + std::to_string(i);
        // cv::imshow(window_name, contour_img);
        // cv::waitKey(0.0);
        std::vector<cv::Point3d> contour_3d_positions;
        for(size_t j=0; j<contour_img.rows; j++)
        {
            for(size_t k=0; k<contour_img.cols; k++)
            {
                int index = j * contour_img.cols + k;
                int pixelValue = (int)contour_img.at<uchar>(j,k);
                if(pixelValue == 255)
                {
                    cv::Point3d pt;
                    pt.x = cloud->points[index].x;
                    pt.y = cloud->points[index].y;
                    pt.z = cloud->points[index].z;
                    if (pt.z > 0.0) contour_3d_positions.push_back(pt);
                }
            }
        }
        // find the central position of the ball
        double xc, yc, zc, rc;
        this->Fit3DSphere(contour_3d_positions, xc, yc, zc, rc);
        if(fabs(rc - radius_list[i])<=2.0)
        {
            cv::Point3d pt_centre;
            pt_centre.x = xc; pt_centre.y=yc; pt_centre.z=zc;
            ball_centre_list[i] = pt_centre;
        }
        else{
            std::cout<<"Ball central position reconstruction failure"<<std::endl;
        }
    }
    centre_ball_1 = {ball_centre_list[0].x, ball_centre_list[0].y, ball_centre_list[0].z};
    centre_ball_2 = {ball_centre_list[1].x, ball_centre_list[1].y, ball_centre_list[1].z};
}

Eigen::Vector3d FeatureDetection::ReconstructJ4Position(const cv::Mat &img_depth, const PointCloudT::Ptr &cloud, const double &ball_1_radius, const double &ball_2_radius)
{
    // Find the centre positions of both red balls
    cv::Mat img_copy = img_depth.clone();
    PointCloudT::Ptr cloud_copy(new PointCloudT);
    cloud_copy->points = cloud->points;
    Eigen::Vector3d centre_ball_1, centre_ball_2;
    this->FindBallCentres(img_copy, cloud_copy, ball_1_radius, ball_2_radius, centre_ball_1, centre_ball_2);
    Eigen::Vector3d j4_position = ((Lbb+Lbp)*centre_ball_2 - centre_ball_1 * Lbp)/Lbb; // unit(mm)
    return j4_position; // unit(mm)
}