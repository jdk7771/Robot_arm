#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include "myrobot/utils.h"

/*
输入是弧度
*/

class Ur5eArm{
    public:
        Ur5eArm(int dof,const Eigen::MatrixXd& poss,const Eigen::MatrixXd& quats,const Eigen::MatrixXd& axiss):
                _dof(dof),_joint_length(poss),_joint_roation_axis(axiss),_joint_quater(quats){
                    _angles = Eigen::VectorXd::Zero(_dof);
        }


        void setBaseWorld(Eigen::Matrix<double,4,4> TBaseWorld){
            _T_world_base = TBaseWorld;
        }

        void setTcpOffset(Eigen::Matrix<double,4,4> T_tcp_offset){
            _T_tcp_offset = T_tcp_offset;
        }
        

        Eigen::Matrix4d getStep(
            const Eigen::Vector3d pos,
            const Eigen::Vector4d quat,
            const Eigen::Vector3d axis = Eigen::Vector3d(0.0,0.0,1.0),
            const double angle = 0.0
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
/*
        是否有偏移 base_word 和 tcp是否有
        没有的话 其实就是 法兰
*/
        Eigen::Matrix4d forward_kinematic(bool base_offset  ,bool tcp_offset ,Eigen::VectorXd angles = {})
            {
                _joint_pos_world.clear();
                Eigen::Matrix4d T_final = Eigen::Matrix4d::Identity();
                if(angles.size() == 0 )
                {
                    angles = _angles;
                }
                if(base_offset ==1 ) T_final = T_final * _T_world_base;
                for (int i = 0; i<_dof; i++)
                { 
                    Eigen::RowVector3d pos = _joint_length.row(i);
                    Eigen::RowVector4d quat = _joint_quater.row(i);
                    Eigen::RowVector3d axis = _joint_roation_axis.row(i);
                
                    Eigen::Matrix4d pose_tmp = Myrobotutils::get_T_joint_axis(pos,quat,axis,angles[i]);
                    T_final = T_final* pose_tmp;
                    _joint_pos_world.push_back(T_final);

                }
                
                if(tcp_offset == 1)  T_final = T_final * _T_tcp_offset;
                _pose_tcp = T_final;
                
                return T_final;
            }

        


        Eigen::MatrixXd get_Jacobian()
        {
            Eigen::Matrix4d T_tcp = _pose_tcp;
            Eigen::MatrixXd  jacobian(6,_dof);
            

            for(int i =0;i<_joint_pos_world.size();i++)
            {
                
                Eigen::Matrix<double,6,1> jaco_tmp;

                Eigen::Matrix4d T_cur = _joint_pos_world[i];
                Eigen::Matrix4d diff = Myrobotutils::get_T_diff(T_tcp, T_cur,1);

                Eigen::Vector3d diff_pos = diff.block<3,1>(0,3);

                Eigen::Vector3d axis = T_cur.block<3,3>(0,0) * _joint_roation_axis.row(i).transpose();
                

                jaco_tmp.block<3,1>(0,0) =  axis.cross(diff_pos);
                jaco_tmp.block<3,1>(3,0) = axis;

                jacobian.col(i) = jaco_tmp;
            }
            return jacobian;

        }
        /*
          读到angles然后写入关节角度
        */
        void write_angles(Eigen::VectorXd angles)
        {
            _angles = angles;
        }

        Eigen::Matrix<double,6,1>  solve_ik(Eigen::Matrix4d tar_Tpos)
        {   Eigen::VectorXd tar_angles = _angles;
            
            double err = 100000;
            while(err>1)
            {
                Eigen::Matrix4d curr_Tpose =  forward_kinematic(1,1,tar_angles);

                Eigen::MatrixXd J = get_Jacobian();
                Eigen::MatrixXd diff = Myrobotutils::get_T_diff(curr_Tpose, tar_Tpos, 0);
                Eigen::VectorXd deta_angles = (J.transpose()*J + lamada*lamada*Eigen::Matrix<double,6,6>::Identity()).inverse()*J.transpose()*diff;
                tar_angles += deta_angles;

            }
            return tar_angles;

        }
        /// @brief  其实 就是加了阻尼之后，求导逆增益在什么时候最大最好
            // 所以这羊
        /// @param tar_Tpos  
        /// @return 
        Eigen::Matrix<double,6,1>  solve_ik_svd(Eigen::Matrix4d tar_Tpos)
        {   Eigen::VectorXd tar_angles = _angles;
            
            double err = 100000;
            double nizuni;
            while(err>1)
            {
                Eigen::Matrix4d curr_Tpose =  forward_kinematic(1,1,tar_angles);

                Eigen::MatrixXd J = get_Jacobian();
                Eigen::MatrixXd diff = Myrobotutils::get_T_diff(curr_Tpose, tar_Tpos);
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J,Eigen::ComputeFullU | Eigen::ComputeFullV);
                auto cigama = svd.singularValues();
                for(int i = 0;i<cigama.size();i++)
                {
                    if (cigama[i]<low) nizuni = low_lamada;
                    else if ( cigama[i]>high) nizuni = high_lamada;
                    else cigama[i] = getzuni(nizuni) ;

                    cigama[i] = cigama[i]/(cigama[i]*cigama[i]+nizuni*nizuni);

                }

                Eigen::VectorXd deta_angles = svd.matrixU()*cigama.transpose()*svd.matrixU().transpose()*diff;
                tar_angles += deta_angles;

            }
            return tar_angles;

        }

        void set_angles(const Eigen::Matrix<double,6,1>& angles) { _angles = angles; }
        Eigen::Matrix<double,6,1> get_angles() const { return _angles; }



    private:
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

    Eigen::VectorXd _angles;
    std::vector<Eigen::Matrix4d> _joint_pos_world;

    //svd阻尼系数
    double lamada = 0.3;
    double high = 1;
    double high_lamada = 0;
    double low = 0.2;
    double low_lamada = 0;
    double getzuni(double ciga)
    {
        return ciga;
    }

};


