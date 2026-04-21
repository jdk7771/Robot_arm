import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
sys.path.append('/home/jiang/snap/Learnrobot/robot/build')
import myrobot_py
import numpy


np.set_printoptions(suppress=True, precision=3, formatter={'float': '{: 0.3f}'.format})
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
# 末端这个想一下
ee_site_name = "wrist_3_link"
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
        data.ctrl[:6] = np.array([0,0,0,0,0,0])

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
            
            for i in range(len(arm._joint_pos_world)):
                print(f"{body_names[i]}:")
                print(arm._joint_pos_world[i])
            print(f"末端位置是{T_final}")
            current_qvel = data.qvel[:6] # 6 个关节的速度 (rad/s)
            

            
            # # 2. 读取笛卡尔空间数据 (Cartesian Space - FK 结果)
            # # xpos 是位移 (x, y, z)，xmat 是旋转矩阵 (3x3)
            # ee_pos = data.site_xpos[ee_id]
            # ee_mat = data.site_xmat[ee_id].reshape(3, 3)
            


# ##调试

#             print(f"\n--- [Time: {data.time:.2f}s] 各关节位姿校验 ---")

#             for i, b_id in enumerate(body_ids):
#                 name_str = body_names[i]
                
#                 # 1. 获取 MuJoCo 的全局位姿 (真值)
#                 xpos = data.xpos[b_id]      # 3维向量
#                 xmat = data.xmat[b_id].reshape(3, 3)  # 3x3 旋转矩阵
                
#                 # 2. 获取你 C++ 库计算的对应关节位姿 (理论值)
#                 # 注意：你的 _joint_pos_world 索引可能和这里差 1 (因为包含了 base 或者顺序不同)
#                 # 这里假设你 C++ 里的 _joint_pos_world 已经通过 forward_kinematic 更新了
#                 try:
#                     my_T = arm.forward_kinematic(1, 1, current_qpos) # 这一步通常更新内部的 _joint_pos_world
#                     # 假设你导出了一个接口 get_internal_pos(index)
#                     # 或者你现在直接看最后的结果
#                 except:
#                     pass

#                 print(f"Link: {name_str}")
#                 print(f"  MuJoCo Pos: {np.round(xpos, 4)}")
#                 # 如果你想看旋转矩阵是否对齐：
#                 print(f"  MuJoCo Mat:\n{np.round(xmat, 3)}")





    
            # print(f"Time: {data.time:.2f}s")
            # print(f"  [Joints] q: {np.round(current_qpos, 3)}")
            # print(f"  [End-Effector] Pos: {np.round(ee_pos, 3)}")
            # print(f"  [Rotation Matrix]:\n{ee_mat}") # 如需验证旋转，取消此行注释
        # 获取 base_link 的 ID
        # base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

        # # 读取底座在世界坐标系的位置
        # base_pos = data.xpos[base_id]
        # base_quat = data.xquat[base_id]

        # print(f"底座在世界系的位置 (Base in World): {base_pos}")
        # print(f"底座在世界系的姿态 (Base Quaternion): {base_quat}")
        # 同步画面并维持实时性
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)