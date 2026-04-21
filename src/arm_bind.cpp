#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include "myrobot/utils.h"
#include "myrobot/arm_bind.h"

/*
输入是弧度
*/


        void Ur5eArm::setBaseWorld(Eigen::Matrix<double,4,4> TBaseWorld){
            _T_world_base = TBaseWorld;
        }

        void Ur5eArm::setTcpOffset(Eigen::Matrix<double,4,4> T_tcp_offset){
            _T_tcp_offset = T_tcp_offset;
        }
        

        Eigen::Matrix4d Ur5eArm::getStep(
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
/*
        是否有偏移 base_word 和 tcp是否有
        没有的话 其实就是 法兰
*/
        Eigen::Matrix4d Ur5eArm::forward_kinematic(bool base_offset  ,bool tcp_offset ,Eigen::VectorXd angles )
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

        


        Eigen::MatrixXd Ur5eArm::get_Jacobian()
        {
            Eigen::Matrix4d T_tcp = _pose_tcp;
            Eigen::MatrixXd  jacobian(6,_dof);
            

            for(int i =0;i<_joint_pos_world.size();i++)
            {
                
                Eigen::Matrix<double,6,1> jaco_tmp;

                Eigen::Matrix4d T_cur = _joint_pos_world[i];
                Eigen::Matrix4d diff = Myrobotutils::get_T_diff(T_cur, T_tcp,1);

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

          在数值计算中，能不求逆就不求逆。inverse() 计算量大且在矩阵接近奇异（Singular）时极其不稳定。

          使用 Eigen 的分解法，比如 .ldlt().solve() 或 
          .colPivHouseholderQr().solve()。对于 DLS 这种对称正定矩阵，LDLT 是最快的。

        */
        void Ur5eArm::write_angles(Eigen::VectorXd angles)
        {
            _angles = angles;
        }

        Eigen::Matrix<double,6,1>  Ur5eArm::solve_ik(Eigen::Matrix4d tar_Tpos,double threshold)
        {   

            int max_iter = 500;
            int iter = 0;
            
            
            Eigen::Matrix<double,6,1> tar_angles = _angles;

            Eigen::Matrix4d curr_Tpose =  forward_kinematic(1,1,tar_angles);
            Eigen::Matrix<double,6,6> J;
            Eigen::Matrix<double,6,1> diff;
            double err = 100000;
            while(err>threshold && iter<max_iter)
            {

                iter++;
                J = get_Jacobian();
                diff = Myrobotutils::get_IK_diff(curr_Tpose, tar_Tpos);
                err = diff.norm();
                Eigen::Matrix<double,6,1> deta_angles = (J.transpose()*J + lamada*lamada*Eigen::Matrix<double,6,6>::Identity()).inverse()*J.transpose()*diff;
                tar_angles += deta_angles;

                curr_Tpose =  forward_kinematic(1,1,tar_angles);
                

            }
            std::cout<<"iter:"<<iter<<std::endl;
            std::cout<<"err:"<<err<<std::endl;

            return tar_angles;

        }
        /// @brief  其实 就是加了阻尼之后，求导逆增益在什么时候最大最好
            // 所以这羊
        /// @param tar_Tpos  
        /// @return 
        Eigen::Matrix<double,6,1>  Ur5eArm::solve_ik_svd(Eigen::Matrix4d tar_Tpos,double threshold)
        {   
            
            int max_iter = 100;
            int iter = 0;
            double sigma_min_threshold = 1e-3;

            Eigen::Matrix<double,6,1> tar_angles = _angles;
            Eigen::Matrix4d curr_Tpose;
            Eigen::Matrix<double,6,1> diff;
            Eigen::MatrixXd J;

            double err = 100000;
            while(err>threshold && iter < max_iter)
            {
                iter++;
                curr_Tpose =  forward_kinematic(1,1,tar_angles);

                J = get_Jacobian();
                diff = Myrobotutils::get_IK_diff(curr_Tpose, tar_Tpos);
                err = diff.norm();
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J,Eigen::ComputeFullU | Eigen::ComputeFullV);


                auto cigama = svd.singularValues();
                for(int i = 0;i<cigama.size();i++)
                {
                    double lambda = (cigama[i] < sigma_min_threshold) ? max_lamada : 0.0;

                    cigama[i] = cigama[i]/(cigama[i]*cigama[i]+lambda*lambda);
                }

                Eigen::Matrix<double,6,1> deta_angles = svd.matrixV()*cigama.asDiagonal()*svd.matrixU().transpose()*diff;
                tar_angles += deta_angles;
                

            }
            std::cout<<"iter:"<<iter<<std::endl;
            std::cout<<"err:"<<err<<std::endl;

            return tar_angles;

        }



