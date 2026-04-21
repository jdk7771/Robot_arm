#ifndef ARM_BIND_H
#define ARM_BIND_H

#include <Eigen/Dense> 
#include <vector>
class Ur5eArm
{
    public:
        Ur5eArm(int dof,const Eigen::MatrixXd& poss,const Eigen::MatrixXd& quats,const Eigen::MatrixXd& axiss):
                _dof(dof),_joint_length(poss),_joint_roation_axis(axiss),_joint_quater(quats) ,_angles(Eigen::Matrix<double,6,1>::Zero()){
                    
                };


        void setBaseWorld(Eigen::Matrix<double,4,4> TBaseWorld);

        void setTcpOffset(Eigen::Matrix<double,4,4> T_tcp_offset);

        Eigen::Matrix4d getStep(
            const Eigen::Vector3d pos,
            const Eigen::Vector4d quat,
            const Eigen::Vector3d axis = Eigen::Vector3d(0.0,0.0,1.0),
            const double angle = 0.0
        );
       
/*
        是否有偏移 base_word 和 tcp是否有
        没有的话 其实就是 法兰
*/
        Eigen::Matrix4d forward_kinematic(bool base_offset  ,bool tcp_offset ,Eigen::VectorXd angles = {});
          
        


        Eigen::MatrixXd get_Jacobian();
    
        /*
          读到angles然后写入关节角度
        */
        void write_angles(Eigen::VectorXd angles);


        Eigen::Matrix<double,6,1>  solve_ik(Eigen::Matrix4d tar_Tpos,double sigma_min_threshold);
 
        /// @brief  其实 就是加了阻尼之后，求导逆增益在什么时候最大最好
            // 所以这羊
        /// @param tar_Tpos  
        /// @return 
        Eigen::Matrix<double,6,1>  solve_ik_svd(Eigen::Matrix4d tar_Tpos,double sigma_min_threshold);

        void set_angles(const Eigen::Matrix<double,6,1>& angles) { _angles = angles; }
        Eigen::Matrix<double,6,1> get_angles() const { return _angles; }

    // private:
    // Eigen::Matrix<double,8,3> poss ;
    // Eigen::Matrix<double,8,4> quats ;
    // Eigen::Matrix<double,8,3> axiss ;
    int _dof ;
    Eigen::MatrixXd _joint_length  ;
    Eigen::MatrixXd _joint_roation_axis ;
    Eigen::MatrixXd _joint_quater ;
    Eigen::Matrix4d _pose_tcp;
    Eigen::Matrix<double,4,4> _T_world_base = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,4,4> _T_tcp_offset = Eigen::Matrix<double,4,4>::Identity();

    Eigen::Matrix<double,6,1> _angles;
    std::vector<Eigen::Matrix4d> _joint_pos_world;

    //svd阻尼系数
    double lamada = 0.3;
    double max_lamada = 0.1;
    double getzuni(double ciga)
    {
        return ciga;
    };

};


#endif