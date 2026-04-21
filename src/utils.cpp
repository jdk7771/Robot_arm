#include <Eigen/Dense>

/*
    获取两个pose的T
    i  0是关于轴的旋转    1是4*4   0是pos+四元素
    默认为轴的旋转  

*/
namespace Myrobotutils
{
Eigen::MatrixXd get_T_diff(Eigen::Matrix4d cur,Eigen::Matrix4d tar, int i)
{
    Eigen::Vector3d curpos = cur.block<3,1>(0,3);
    Eigen::Vector3d tarpos = tar.block<3,1>(0,3);
    Eigen::Vector3d error = tarpos-curpos;

    Eigen::Matrix3d curroa = cur.block<3,3>(0,0);
    Eigen::Matrix3d tarroa = tar.block<3,3>(0,0);
    Eigen::Matrix3d errroa = tarroa*curroa.transpose();

    
    // Eigen::Quaterniond q(errroa);
    // q.normalize();

    // Eigen::Matrix<double,7,1> result;
    // result.block<1,3>(0,0) = error;
    // result.block<1,4>(3,0) = q.coeffs();

    Eigen::Matrix<double,4,4> result;
    result.block<3,1>(0,3) = error;
    result.block<3,3>(0,0) = errroa;

    //转为关于轴的旋转
    Eigen::AngleAxisd arree(errroa);
    Eigen::Matrix<double,6,1> res ;
    res.block<3,1>(0,0) =  error;
    res.block<3,1>(3,0) = arree.angle() * arree.axis();

    if(i == 1) return result;
    else if (i==0)
    {
       return  res;
    }
    
    

}   
Eigen::Matrix<double,6,1> get_IK_diff(Eigen::Matrix4d cur,Eigen::Matrix4d tar)
{
    Eigen::Vector3d curpos = cur.block<3,1>(0,3);
    Eigen::Vector3d tarpos = tar.block<3,1>(0,3);
    Eigen::Vector3d error = tarpos-curpos;

    Eigen::Matrix3d curroa = cur.block<3,3>(0,0);
    Eigen::Matrix3d tarroa = tar.block<3,3>(0,0);
    Eigen::Matrix3d errroa = tarroa*curroa.transpose();

    
    // Eigen::Quaterniond q(errroa);
    // q.normalize();

    // Eigen::Matrix<double,7,1> result;
    // result.block<1,3>(0,0) = error;
    // result.block<1,4>(3,0) = q.coeffs();

    Eigen::Matrix<double,4,4> result;
    result.block<3,1>(0,3) = error;
    result.block<3,3>(0,0) = errroa;

    //转为关于轴的旋转
    Eigen::AngleAxisd arree(errroa);
    Eigen::Matrix<double,6,1> res ;
    res.block<3,1>(0,0) =  error;
    res.block<3,1>(3,0) = arree.angle() * arree.axis();

    return  res;

}   

// #给定坐标系 旋转后的坐标
Eigen::Matrix4d get_T_joint_axis(
    const Eigen::Vector3d pos,
    const Eigen::Vector4d quat,
    const Eigen::Vector3d axis,
    const double angle 
)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4,4);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(3,3);
    
    Eigen::Quaterniond q(quat[0],quat[1],quat[2],quat[3]);
    q.normalize();

    R = q.toRotationMatrix();

    Eigen::Matrix3d Ta ;
    Ta = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
    T.block<3,3>(0,0) = R*Ta;
    T.block<3,1>(0,3) = pos;
    
    return T;

}
}