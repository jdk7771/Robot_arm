import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
sys.path.append('/home/jiang/snap/Learnrobot/robot_arm/build')
import myrobot_py
import numpy


np.set_printoptions(suppress=True, precision=3, formatter={'float': '{: 0.5f}'.format})
dof = 6

body_names = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]

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
                  [1,0,0,0]],dtype=np.float64)

arm = myrobot_py.Ur5eArm(dof,poss,quats,axiss)

Tbasepos = np.array([[0,0,0]]).transpose()
Tbaseaxis = np.array([[0,1,0]]).transpose()
Tbasequat = np.array([[0,0,0,-1]]).transpose()

# Ttcppos = np.array([[0,0.1,0]]).transpose()
# Ttcpaxis = np.array([[0,1,0]]).transpose()
# Ttcpquat = np.array([[-1,1,0,0]]).transpose()
Ttcppos = np.array([0, 0.1, 0], dtype=np.float64)
# 注意：MuJoCo 的 site quat "-1 1 0 0" 对应的 w=-1, x=1, y=0, z=0
Ttcpquat = np.array([-1, 1, 0, 0], dtype=np.float64) 
Ttcpaxis = np.array([0, 0, 1], dtype=np.float64) # TCP 本身不旋转，轴设为默认 Z


TbaseWorld = myrobot_py.get_T_joint_axis(Tbasepos,Tbasequat,Tbaseaxis,0)
Ttcp = myrobot_py.get_T_joint_axis(Ttcppos,Ttcpquat,Ttcpaxis,0)

arm.setBaseWorld(TbaseWorld)
arm.setTcpOffset(Ttcp)


print("传奇姜大昆手臂已就绪")
# 1. 加载模型 (请根据你的实际路径修改)
model_path = "/home/jiang/tools/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
model = mujoco.MjModel.from_xml_path(model_path)

# 重力 归0
model.opt.gravity[:] = [0, 0, 0]


data = mujoco.MjData(model)

# 2. 预先获取末端 Site 的 ID (用于 FK 校验)
# UR5e Menagerie 模型通常定义了 "attachment_site" 作为末端

ee_site_name = "attachment_site"

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)



body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in body_names]



# 3. 启动可视化界面
with mujoco.viewer.launch_passive(model, data) as viewer:
    print(f"--- UR5e 仿真已启动 ---")
    while viewer.is_running():
        step_start = time.time()

        # -------------------------------------------------------
        # 【控制部分 - Control】
        # data.ctrl 对应 XML 中定义的 actuators。对于 UR5e，通常是 6 个位置控制
        # -------------------------------------------------------
        # 示例：让 6 个关节呈正弦摆动（验证动态响应）
        # target_q = 0.5 * np.sin(data.time * 2.0)
        data.ctrl[:6] = np.array([0.1,0.2,0.1,0.13,0.32,0.1])

        # -------------------------------------------------------
        # 【仿真步进 - Physics Step】
        # -------------------------------------------------------
        mujoco.mj_step(model, data)

        # -------------------------------------------------------
        # 【状态读取 - Reading / Sensing】
        # -------------------------------------------------------
        if data.time % 0.5 < 0.002: # 每 0.5 秒采样输出一次
            # 1. 读取关节空间数据 (Joint Space)
            current_qpos = data.qpos[:6] # 6 个关节的角度 (rad)
       
            T_final = arm.forward_kinematic(1,1,current_qpos)

            # print(f"jdkFK_末端位置是{T_final}")
            
            
#             # # 2. 读取笛卡尔空间数据 (Cartesian Space - FK 结果)
#             # # xpos 是位移 (x, y, z)，xmat 是旋转矩阵 (3x3)
            ee_pos = data.site_xpos[ee_id]
            ee_mat = data.site_xmat[ee_id].reshape(3, 3)

            target_t = np.eye(4)
            target_t[0:3,0:3] = ee_mat
            target_t[0:3, 3] = ee_pos
            # print(f"  [End-Effector] Pos: {np.round(ee_pos, 3)}")
            # print(f"  [Rotation Matrix]:\n{ee_mat}") # 如需验证旋转，取消此行注释
            
            arm.write_angles(np.array([0.1,0.2,0.1,0.33,0.32,0.1]))
            angle = arm.solve_ik(target_t,1e-3)


            T1 = arm.forward_kinematic(1,1,angle)
            T2 = arm.forward_kinematic(1,1,data.qpos[:6])

            print(f"jdk_angles{angle}")
            print(f"real_angle{data.qpos[:6]}")
            print()
            print(T1)
            print(T2)
            
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)