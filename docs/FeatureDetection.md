# A detailed explnation for class FeatureDetection
This markdown file provides a detailed description of the functions declared within class ```FeatureDetection```, an object that encapsulates computer vision algorithms used in this project.

## Private variables & functions
* K_int
it represents the intrinsic matrix for the RGB lens of the camera
* fx, fy, cx, cy 
Intrinsic camera parameters that comprise K_int.

* Tcr, Trc
$4 \times 4$ homogenous transformation matrices between the camera and robot frame. 

* Lbb
It represents the distance between the centre of two spherical markers when they are placed on the tool shaft. The value is 0.050 (m) by default

* Lbp
It represents the distance between the centre of the smaller marker ball and joint 4 when it is places on the tool shaft. The value is 0.017 (m) by default.

* T_depth2RGB_Acusense
The extrinsic matrix that transforms the depth frame to the colour frame of the Acusense camera.

* ContourDetection
```
void ContourDetection(const cv::Mat image, int min_area, std::vector<cv::Mat> &contour_img_list)
```
This function reads input image and detect circular contours, the size of which is larger than ```min_area```. Output ```contour_img_list``` is a vector of images overlaid with detected circular contours, in the descending order of contour areas. 

First, we leverage the built-in OpenCV function ```cv::findContours``` to detect all circular contours in the image.
```cpp
    contour_img_list.clear();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
```
Then we rearrange the detected contours in an descending order
```cpp
    std::sort(contours.begin(), contours.end(),[](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2)
              {return cv::contourArea(c1,false) > cv::contourArea(c2, false);}); 
```
Finally, we manually overlay these detected contours on the original image.
```cpp
    for(int i=0; i<contours.size(); i++)
    { 
        if(cv::contourArea(contours[i]) > min_area)
        {
            auto area = cv::contourArea(contours[i]);
            cv::Mat image_with_contour = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
            cv::drawContours(image_with_contour, contours, i, cv::Scalar(255), -1, 8,hierarchy);
            contour_img_list.push_back(image_with_contour);
        }
    }
```
* Fit3DSphere
```cpp
void Fit3DSphere(const std::vector<cv::Point3d> &pt_list, double &xc, double &yc, double &zc, double &rc);
```
This function aims to find the 3D position of a spherical ball centre and the radius of the ball, given 3D positions of points on the spherical surface.  

Assume there are $i$ points on the surface, with their 3D positions $P_i=(x_i,y_i,z_i)$. The centre of the spherical ball with a radius $r_c$ is $P_c=(x_c,y_c,z_c)$. The following relationship is satisfied $(x_i-x_c)^2+(y_i-y_c)^2+(z_i-z_c)^2=r^2_c$. By rearranging the equation, we have $x_i \times c_0+y_i \times c_1 + z_i \times c_2 + 1 \times c_3 = x^2_i + y^2_i + z^2_i$, where $c_0=\frac{1}{2}x_c, c_1=\frac{1}{2}y_c,c_2=\frac{1}{2}z_c, c_3=r^2_c-x^2_c-y^2_c-z^2_c$. Hence, we can calculate $x_c, y_c, z_c$ and $r_c$ after collecting a list of $P_i$. The code goes as follows.
```cpp
    int n_pts = pt_list.size();
    Eigen::MatrixXd A(n_pts, 4), b(n_pts,1);
    for(size_t i=0; i<n_pts; i++)
    {
        double x = pt_list[i].x, y = pt_list[i].y, z = pt_list[i].z;
        A(i,0) = x; A(i,1) = y; A(i,2) = z; A(i,3) = 1;
        b(i,0) = x*x + y*y + z*z;
    }
    Eigen::VectorXd c = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    // Eigen::VectorXd c = A.colPivHouseholderQr().solve(b);
    xc = c[0]/2; yc = c[1]/2; zc = c[2]/2;
    rc = sqrt(c[3]+xc*xc+yc*yc+zc*zc);
```
* FindBallCentres
```cpp
void FindBallCentres(cv::Mat img, PointCloudT::Ptr cloud, double radius_ball_1, double radius_ball_2, Eigen::Vector3d &centre_ball_1, Eigen::Vector3d &centre_ball_2)
```
This function builds upon the prvious functions `ContourDetection` and `Fit3DSphere`, aiming to find the centre positions of both red spherical markers given the depth image of the current scene. 

First, we use the function `ContourDetection` to generate binary images with overlaid contours, where pixels within the contour are coloured in white. We also conduct a sanity check to make sure there are at least two circular contours detected in each frame.
```cpp
    // Step 1, find contours
    std::vector<cv::Mat> contour_img_list;
    std::vector<double> radius_list = {radius_ball_1, radius_ball_2}; // unit (mm), r1 > r2
    std::vector<cv::Point3d> ball_centre_list(2);
    int min_area = 400;
    this->ContourDetection(img, min_area, contour_img_list);
    assert(contour_img_list.size() >= 2 && "Not enough ball contours detected");
```

Second, we aim to extract the 3D position of points on the spherical surface, the 2D projection of which are pixels confined within the detected contour. Our method is to look up their 3D position from the depth image. The 3D position of the detected surface points are stored in `contour_3d_positions`, which are then sent into  `Fit3DSphere` for calculating the centre position of two spherical markers.
```cpp
        cv::Mat contour_img = contour_img_list[i];
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
```
To ensure the accuracy of the calculations, we have added a block for sanity check. The criteria is that the reconstructed radii of markers should be within a 2mm range from the ground truth values.

```cpp
        if(fabs(rc - radius_list[i])<=2.0)
        {
            cv::Point3d pt_centre;
            pt_centre.x = xc; pt_centre.y=yc; pt_centre.z=zc;
            ball_centre_list[i] = pt_centre;
        }
        else{
            std::cout<<"Ball central position reconstruction failure"<<std::endl;
        }
```

Finally, we have obtained `centre_ball_1` and `centre_ball_2` for the reconstructed centre position of both markers.

* world2pixel
```cpp
void world2pixel(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D);
```
This function projects a 3D point onto the image plane via projection geometry, with given intrinsic camera parameters. 

Assume there is no distortion factor, we aim to project 3D position `point_3D` to its pixel coordinate `pixel_2D`. 
```cpp
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
```

* world2pixel_Acusense
```cpp
void world2pixel_Acusense(const Eigen::Vector3d &point_3D, Eigen::Vector2d &pixel_2D)
```
This function projects a 3D point expressed in the depth frame onto the colour image plane. It first transforms the 3D point to the colour frame via `T_depth2RGB_Acusense` before projecting the 3D position onto the colour image plane via `world2pixel`. 

## Public functions
* FeatureDetection
For class declaration

* ReadIntrinsicMatrix
```cpp
void ReadIntrinsicMatrix(const Eigen::MatrixXd &Intrinsic_mat)
```
This function reads the intrinsic matrix of a camera.

* ReadHandEyeTransform
```cpp
void ReadHandEyeTransform(const Eigen::Matrix4d &T)
```
This function reads the hand-eye transformation matrix between the camera and robot base frame. 

* ReadAcusenseDepth2RGBMat
```cpp
void ReadAcusenseDepth2RGBMat(const Eigen::Matrix4d &T_d_rgb)
```
This function reads the extrinsic matrix between the depth and colour frame of the Acusense camera. 

* ReadPointCloudPCD
```cpp
void ReadPointCloudPCD(const std::string &file_path, const int &n_rows, const int &n_cols, cv::Mat &img, PointCloudT::Ptr &cloud)
```
This functions reads point cloud from a local file and reshapes the point cloud into a `n_rows` $\times$ `n_cols` binary image, where pixels that have non-zero depth values are coloured in white, otherwise black. 

* cvtDepth2Colour_Acusense
```cpp
void cvtDepth2Colour_Acusense(const Eigen::Vector3d &pt_depth, Eigen::Vector3d &pt_colour)
```
This function converts a 3D point expressed in the depth frame to the colour frame via the extrinsic matrix `T_depth2RGB_Acusense`.

* cvt2cameraFrame
```cpp
void cvt2cameraFrame(const Eigen::Vector3d &point_robot, Eigen::Vector3d &point_camera);
```
This function converts a 3D expressed in the robot frame to the camera frame through the hand-eye transformation matrix.

* ReconstructJ4Position
```cpp
Eigen::Vector3d ReconstructJ4Position(const cv::Mat &img_depth, const PointCloudT::Ptr &cloud, const double &ball_1_radius, const double &ball_2_radius)
```
This public function serves as a wrapper of several private functions for users to reconstruct the 3D position of joint 4. 
The pipeline goes as follows
1. Calculate the position of both markers in the depth camera frame through `FindBallCentres`. 
2. Find out joint 4 position via simple linear algebra.


* drawShaftAxisColourAcusense
```cpp
cv::Mat drawShaftAxisColourAcusense(cv::Mat img, std::string window_name, const Eigen::Vector3d &origin, const Eigen::Vector3d &end_pt)
```
This function overlays the back-projected tool axis onto the image plane. The central tool axis is defined by two points on the axis. We selected the RCM point and joint 4 in this project. The idea is to first convert both the RCM point and joint 4 from the robot base frame to the depth frame of the camera via `cvt2cameraFrame`. Then we transform both points to the colour frame of the camera via `cvtDepth2Colour_Acusense`. Finally, we back project both points onto the colour image plane via `world2pixel` and connect these two points using a dashed line.
