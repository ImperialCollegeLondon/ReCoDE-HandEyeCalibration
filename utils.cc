#include "utils.hpp"

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