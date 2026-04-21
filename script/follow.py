import mujoco
import mujoco.viewer
import numpy as np
import time
import sys

# 1. 导入你编译好的高速 Release 版 C++ 库
sys.path.append('/home/jiang/snap/Learnrobot/robot_arm/build')
import myrobot_py

# 打印配置
np.set_printoptions(suppress=True, precision=4)

# --- 机器人几何参数 (基于你之前的设定) ---
dof = 6
poss = np.array([[0, 0, 0.163], [0, 0.138,0], [0, -0.131,0.425], 
                 [0,0,0.392], [0,0.127,0], [0,0,0.1]], dtype=np.float64)
axiss = np.array([[0,0,1], [0,1,0], [0,1,0], [0,1,0], [0,0,1], [0,1,0]], dtype=np.float64)
quats = np.array([[1,0,0,0], [1,0,1,0], [1,0,0,0], [1,0,1,0], [1,0,0,0], [1,0,0,0]], dtype=np.float64)

# --- 初始化自定义 Arm 类 ---
arm = myrobot_py.Ur5eArm(dof, poss, quats, axiss)
TbaseWorld = myrobot_py.get_T_joint_axis(np.zeros(3), np.array([0,0,0,-1]), np.array([0,1,0]), 0)
Ttcp = myrobot_py.get_T_joint_axis(np.array([0, 0.1, 0]), np.array([-1, 1, 0, 0]), np.array([0, 0, 1]), 0)
arm.setBaseWorld(TbaseWorld)
arm.setTcpOffset(Ttcp)

# --- 加载 MuJoCo 模型 ---
model_path = "/home/jiang/tools/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
model = mujoco.MjModel.from_xml_path(model_path)
model.opt.gravity[:] = [0, 0, 0] # 暂时失重环境，专注于运动学验证
data = mujoco.MjData(model)
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")

print("🔥 传奇姜大昆实时控制系统启动...")

# --- 变量初始化 ---
smooth_q = data.qpos[:6].copy() # 平滑后的指令角度
alpha = 0.15 # 平滑系数 (0-1)，越大越灵敏，越小越丝滑

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # -------------------------------------------------------
        # 1. 生成随时间变化的目标位姿 (例如让末端画个小圆圈)
        # -------------------------------------------------------
        t = data.time
        center_pos = np.array([-0.5, -0.2, 0.3]) # 圆心位置
        target_pos = center_pos + np.array([
            0.1 * np.cos(t * 1.5),  # X 轴摆动
            0.1 * np.sin(t * 1.5),  # Y 轴摆动
            0.05 * np.cos(t * 0.5)  # Z 轴微调
        ])
        
        # 保持固定的下朝向姿态 (这里的旋转矩阵根据你之前的 FK 结果调整)
        target_t = np.eye(4)
        target_t[:3, 3] = target_pos
        # 使用一个固定的姿态，比如让末端垂直向下
        target_t[:3, :3] = np.array([[ 0.0, -1.0,  0.0],
                                     [-1.0,  0.0,  0.0],
                                     [ 0.0,  0.0, -1.0]])

        # -------------------------------------------------------
        # 2. 调用你写的 C++ 高速 IK 接口
        # -------------------------------------------------------
        # 使用当前关节角作为初值，保证收敛的连续性
        arm.write_angles(data.qpos[:6]) 
        # 设定一个合理的精度阈值
        ik_q = arm.solve_ik(target_t, 1e-4).ravel()

        # -------------------------------------------------------
        # 3. 控制平滑处理 (非常重要！防止电机由于阶跃指令过热或震荡)
        # -------------------------------------------------------
        # 低通滤波公式: y(n) = (1-a)*y(n-1) + a*x(n)
        smooth_q = (1 - alpha) * smooth_q + alpha * ik_q

        # -------------------------------------------------------
        # 4. 下发指令并步进仿真
        # -------------------------------------------------------
        data.ctrl[:6] = smooth_q
        mujoco.mj_step(model, data)

        # 每 1 秒打印一次状态，不影响性能
        if int(t * 10) % 10 == 0:
            err = np.linalg.norm(data.site_xpos[ee_id] - target_pos)
            print(f"Time: {t:.1f}s | Target Error: {err*1000:.2f} mm")

        viewer.sync()
        # 严格控制帧率
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)