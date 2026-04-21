#include <pybind11/pybind11.h>
#include "myrobot/arm_bind.h"
#include "myrobot/utils.h"
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(myrobot_py,m){
    m.doc() =  "传奇姜大昆的手臂";

    m.def("get_T_diff",&Myrobotutils::get_T_diff,"get diff");

    m.def("get_T_joint_axis", &Myrobotutils::get_T_joint_axis, "get_");

    py::class_<Ur5eArm>(m,"Ur5eArm")
    .def(py::init<int ,const Eigen::MatrixXd&,const Eigen::MatrixXd&,const Eigen::MatrixXd&>(),
            py::arg("dof"),
            py::arg("poss"),
            py::arg("quats"), 
            py::arg("axiss"))

        .def_readwrite("_joint_pos_world", &Ur5eArm::_joint_pos_world)
        .def_readwrite("_angles", &Ur5eArm::_angles)
        .def("setBaseWorld", &Ur5eArm::setBaseWorld, py::arg("TBaseWorld"),
                    "设置世界坐标系到基座的变换矩阵")

        .def("setTcpOffset", &Ur5eArm::setTcpOffset, py::arg("T_tcp_offset"),
             "设置工具末端偏置")

        // 重点：处理 forward_kinematic 的默认参数 angles
        .def("forward_kinematic", &Ur5eArm::forward_kinematic, 
             py::arg("base_offset"), 
             py::arg("tcp_offset"), 
             py::arg("angles") = Eigen::VectorXd(), // 对应 C++ 的 {}
             "正运动学求解")

        .def("get_Jacobian", &Ur5eArm::get_Jacobian, "获取当前雅可比矩阵")

        .def("write_angles", &Ur5eArm::write_angles, py::arg("angles"), "更新关节角度")

        .def("solve_ik", &Ur5eArm::solve_ik, py::arg("tar_Tpos"),py::arg("threshold"), "数值逆解 (Damped Least Squares)")

        .def("solve_ik_svd", &Ur5eArm::solve_ik_svd, py::arg("tar_Tpos"),py::arg("threshold"), "数值逆解 (SVD 阻尼调整)")
        .def("set_angles", &Ur5eArm::set_angles, py::arg("angles"))

        .def("get_angles", &Ur5eArm::get_angles);
}

