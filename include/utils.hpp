#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/// Read in txt file and store data as an Eigen matrix
/// @param path path to the .txt file
/// @param output output matrix
void ReadMatrixFromTxt(const std::string path, Eigen::MatrixXd &output);

/**
 * @brief Obtain joint 4 position expressed in the robot base frame via Denavit–Hartenberg (DH) transformation matrices 
 * 
 * @param JointAngleList Historical joint angles from the encoder reading.
 * @param J4PositionList Output matrix that stores historical joint 4 positions in the robot frame. 
 * @param l1 built-in parameter l1
 * @param l2 built-in parameter l2
 */
void GetJoint4Position(const Eigen::MatrixXd &JointAngleList, Eigen::MatrixXd &J4PositionList, double l1=0.4318, double l2=0.4162);

// Decompose a random matrix A into A = USV'
void SVD(const Eigen::MatrixXd &A, Eigen::MatrixXd &U, Eigen::MatrixXd &S, Eigen::MatrixXd &V);

/**
 * @brief Find out the rigid transformation matrix from one point cloud to the other, with known correspondences
 * 
 * @param pts1 n*3 matrix, which stores the 3D positions of points in cloud 1
 * @param pts2 n*3 matrix, which stores the 3D positions of points in cloud 2
 * @return Eigen::Matrix4d rigid homogeneous transformation matrix 
 */
Eigen::Matrix4d SVD_rigid_transform(const Eigen::MatrixXd &pts1, const Eigen::MatrixXd &pts2);

/**
 * @brief Function for drawing a dashed line
 * 
 * @param img image on which the dashed line is drawn
 * @param pt1 pixel position of the starting point
 * @param pt2 pixel position of the ending point
 * @param color colour scalar
 * @param thickness thickness value
 * @param style line style, "dotted" by default
 * @param gap gap of the dashed line
 */
void DrawDashedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2,
                    cv::Scalar color, int thickness, std::string style,
                    int gap);

#endif