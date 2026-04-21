import mujoco
import numpy as np
import sys
import os

# Add build directory to path
sys.path.append(os.path.abspath('build'))
import myrobot_py

dof = 6

# Data from get_angle.py
poss = np.array([[0, 0, 0.163],
                 [0, 0.138,0],
                [0, -0.131,0.425],
                [0,0,0.392],
                [0,0.127,0],
                [0,0,0.1]
                 ],dtype=np.float64)
axiss = np.array([[0,0,1],
                   [0,1,0],
                   [0,1,0],
                   [0,1,0],
                   [0,0,1],
                   [0,1,0]],dtype=np.float64)
quats = np.array([[1,0,0,0],
                  [1,0,1,0],
                  [1,0,0,0],
                  [1,0,1,0],
                  [1,0,0,0],
                  [1,0,0,1]],dtype=np.float64)

arm = myrobot_py.Ur5eArm(dof,poss,quats,axiss)

Tbasepos = np.array([[0,0,0]]).transpose()
Tbaseaxis = np.array([[0,1,0]]).transpose()
Tbasequat = np.array([[0,0,0,-1]]).transpose()

Ttcppos = np.array([[0,0.1,0]]).transpose()
Ttcpaxis = np.array([[0,1,0]]).transpose()
Ttcpquat = np.array([[-1,1,0,0]]).transpose()

TbaseWorld = myrobot_py.get_T_joint_axis(Tbasepos,Tbasequat,Tbaseaxis,0)
Ttcp = myrobot_py.get_T_joint_axis(Ttcppos,Ttcpquat,Ttcpaxis,0)

arm.setBaseWorld(TbaseWorld)
arm.setTcpOffset(Ttcp)

# Load model
model_path = "ur5e.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

ee_site_name = "attachment_site"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)

# Test with some angles
test_angles = np.array([0.1, 0.2, 0.1, 0.2, 0.1, 0.05])
data.qpos[:6] = test_angles
mujoco.mj_forward(model, data)

# MuJoCo FK
ee_pos_mujoco = data.site_xpos[ee_id]
ee_mat_mujoco = data.site_xmat[ee_id].reshape(3, 3)

# Custom FK
T_final = arm.forward_kinematic(1, 1, test_angles)
ee_pos_custom = T_final[:3, 3]
ee_mat_custom = T_final[:3, :3]

print(f"Joints: {test_angles}")
print(f"MuJoCo Pos: {ee_pos_mujoco}")
print(f"Custom Pos: {ee_pos_custom}")
print(f"Diff Pos: {ee_pos_custom - ee_pos_mujoco}")

print("\nMuJoCo Rot:\n", ee_mat_mujoco)
print("Custom Rot:\n", ee_mat_custom)
print("Diff Rot:\n", ee_mat_custom - ee_mat_mujoco)
