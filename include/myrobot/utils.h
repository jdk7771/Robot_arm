#ifndef MY_UTILS_H
#define MY_UTILS_H
#include <Eigen/Dense>
namespace Myrobotutils
{

    Eigen::MatrixXd get_T_diff(Eigen::Matrix4d cur,Eigen::Matrix4d tar, int i =0);

    Eigen::Matrix4d get_T_joint_axis(
    const Eigen::Vector3d pos,
    const Eigen::Vector4d quat,
    const Eigen::Vector3d axis = Eigen::Vector3d(0.0,0.0,1.0),
    const double angle = 0.0);

    Eigen::Matrix<double,6,1> get_IK_diff(Eigen::Matrix4d cur,Eigen::Matrix4d tar);
};



#endif
