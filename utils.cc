#include "utils.hpp"

void ReadMatrixFromTxt(const std::string path, Eigen::MatrixXd &output)
{
    std::ifstream file(path);
    std::string line, word;
    std::vector<double> row_vec;
    std::vector<std::vector<double>> matrix_input;
    if(!file) 
    {
        std::cout<<"reference file cannot be read in"<<std::endl;
        return;
    }
    int row_count = 0, n_cols;
    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        int count = 0;
        while(std::getline(ss, word, ' '))
        {
            row_vec.push_back(std::stod(word));
            count++;
        }
        row_count++;
        matrix_input.push_back(row_vec);
        n_cols = row_vec.size();
        row_vec.clear();
        // std::cout<<"The "<<line_count<<" th Joint angle = "<<JointAngleReadingList[line_count-1]<<std::endl;
    }
    output.resize(row_count, n_cols);
    for(size_t i=0; i<row_count; i++)
    {
        for(size_t j=0; j<n_cols; j++)
        {
            output(i,j) = matrix_input[i][j];
        }
    }
}

void GetJoint4Position(const Eigen::MatrixXd &JointAngleList, Eigen::MatrixXd &J4PositionList, double l1, double l2)
{
    int n_rows = JointAngleList.rows();
    J4PositionList.resize(n_rows, 3);
    for(size_t i=0; i<n_rows; i++)
    {
        double q1 = JointAngleList(i,0), q2 = JointAngleList(i,1), q3 = JointAngleList(i,2),
               x = l2*sin(q1)*cos(q2) - (l1 - q3)*sin(q1)*cos(q2),
               y = -l2*sin(q2) + (l1 - q3)*sin(q2),
               z = -l2*cos(q1)*cos(q2) + (l1 - q3)*cos(q1)*cos(q2);
        J4PositionList.row(i) << x, y, z;
    }
}

void SVD(const Eigen::MatrixXd &A, Eigen::MatrixXd &U, Eigen::MatrixXd &S, Eigen::MatrixXd &V)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Preserve the shape of U,S,V matrices
    U.resize(A.rows(), A.rows());
    S.resize(A.rows(), A.rows());
    S.setZero(); 
    V.resize(A.cols(), A.cols());
    // Populate U,S,V matrices
    U = svd_solver.matrixU();
    V = svd_solver.matrixV();
    Eigen::VectorXd sigma_list = svd_solver.singularValues();
    for(size_t i=0; i < sigma_list.size(); i++)
    {
        S(i,i) = sigma_list[i];
    }
}

Eigen::Matrix4d SVD_rigid_transform(const Eigen::MatrixXd &pts1, const Eigen::MatrixXd &pts2)
{
    int n_pts1 = pts1.rows(), n_pts2 = pts2.rows();
    if(n_pts1 != n_pts2)
    {
        throw std::invalid_argument("pts1 size doesn't match with pts2");
    }

    Eigen::MatrixXd pts1_mat(n_pts1, 3), pts2_mat(n_pts2, 3);
    for(size_t i=0; i<n_pts1; i++)
    {
        pts1_mat.row(i) << pts1(i,0), pts1(i,1), pts1(i,2);
        pts2_mat.row(i) << pts2(i,0), pts2(i,1), pts2(i,2);
    }

    Eigen::VectorXd mean_1 = pts1_mat.colwise().mean(), 
                    mean_2 = pts2_mat.colwise().mean();
    pts1_mat.rowwise() -= mean_1.transpose();
    pts2_mat.rowwise() -= mean_2.transpose();
    // establish covariance matrix
    auto H = pts1_mat.transpose() * pts2_mat;
    Eigen::MatrixXd U,S,V; // 3*3 matrices
    SVD(H,U,S,V);
    auto R = V * U.transpose();
    auto t = -R * mean_1 + mean_2;

    // Homogeneous transformation matrix
    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    T_12.topLeftCorner(3,3) = R.topLeftCorner(3,3);
    T_12.topRightCorner(3,1) = t.leftCols(1);
    return T_12;
}

void DrawDashedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2,
                    cv::Scalar color, int thickness, std::string style,
                    int gap) {
  float dx = pt1.x - pt2.x;
  float dy = pt1.y - pt2.y;
  float dist = std::hypot(dx, dy);

  std::vector<cv::Point> pts;
  for (int i = 0; i < dist; i += gap) {
    float r = static_cast<float>(i / dist);
    int x = static_cast<int>((pt1.x * (1.0 - r) + pt2.x * r) + .5);
    int y = static_cast<int>((pt1.y * (1.0 - r) + pt2.y * r) + .5);
    pts.emplace_back(x, y);
  }

  int pts_size = pts.size();

  if (style == "dotted") {
    for (int i = 0; i < pts_size; ++i) {
      cv::circle(img, pts[i], thickness, color, -1);
    }
  } else {
    cv::Point s = pts[0];
    cv::Point e = pts[0];

    for (int i = 0; i < pts_size; ++i) {
      s = e;
      e = pts[i];
      if (i % 2 == 1) {
        cv::line(img, s, e, color, thickness);
      }
    }
  }
}