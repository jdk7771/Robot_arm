import mujoco
import mujoco.viewer
import numpy as np
import time
import sys

# 确保路径正确
sys.path.append('/home/jiang/snap/Learnrobot/robot_arm/build')
import myrobot_py

# 优化打印格式
np.set_printoptions(suppress=True, precision=5)

def test_ik_performance(arm, target_t, current_q):
    """
    专门测试 IK 性能的函数
    """
    # 1. 记录开始时间 (使用 perf_counter 精度更高)
    start_time = time.perf_counter()
    
    # 2. 调用逆解
    arm.write_angles(current_q)
    ik_angles = arm.solve_ik_svd(target_t, 1e-4) # 精度阈值 1e-4
    
    # 3. 记录结束时间
    end_time = time.perf_counter()
    duration_ms = (end_time - start_time) * 1000
    
    # 4. 验证精度：用 IK 算出的角度跑一次 FK
    res_t = arm.forward_kinematic(1, 1, ik_angles)
    
    # 计算位置误差 (欧氏距离)
    pos_error = np.linalg.norm(target_t[:3, 3] - res_t[:3, 3])
    
    # 计算姿态误差 (矩阵差的 Frobenius 范数)
    ori_error = np.linalg.norm(target_t[:3, :3] - res_t[:3, :3])
    
    return ik_angles, duration_ms, pos_error, ori_error

# --- 机器人初始化参数 ---
dof = 6
poss = np.array([[0, 0, 0.163], [0, 0.138,0], [0, -0.131,0.425], [0,0,0.392], [0,0.127,0], [0,0,0.1]], dtype=np.float64)
axiss = np.array([[0,0,1], [0,1,0], [0,1,0], [0,1,0], [0,0,1], [0,1,0]], dtype=np.float64)
quats = np.array([[1,0,0,0], [1,0,1,0], [1,0,0,0], [1,0,1,0], [1,0,0,0], [1,0,0,0]], dtype=np.float64)

arm = myrobot_py.Ur5eArm(dof, poss, quats, axiss)
TbaseWorld = myrobot_py.get_T_joint_axis(np.zeros(3), np.array([0,0,0,-1]), np.array([0,1,0]), 0)
Ttcp = myrobot_py.get_T_joint_axis(np.array([0, 0.1, 0]), np.array([-1, 1, 0, 0]), np.array([0, 0, 1]), 0)
arm.setBaseWorld(TbaseWorld)
arm.setTcpOffset(Ttcp)

# --- MuJoCo 初始化 ---
model_path = "/home/jiang/tools/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
model = mujoco.MjModel.from_xml_path(model_path)
model.opt.gravity[:] = [0, 0, 0]
data = mujoco.MjData(model)
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")

print("🚀 IK 性能基准测试启动 (目标样本量: 20)...")

# --- 测试统计变量初始化 ---
max_tests = 5
current_test = 0
sum_dt = 0.0
sum_p_err = 0.0
sum_o_err = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    last_test_time = 0
    target_q = np.array([0.1, -1.0, 1.0, -0.5, 0.3, 0.1]) # 初始化
    
    while viewer.is_running():
        step_start = time.time()
        
        # 维持基础姿态或执行 IK 结果
        data.ctrl[:6] = target_q
        mujoco.mj_step(model, data)

        # 每隔 2 秒进行一次 IK 性能采样
        if data.time - last_test_time > 0.5:
            last_test_time = data.time
            
            current_pos = data.site_xpos[ee_id].copy()
            current_mat = data.site_xmat[ee_id].reshape(3, 3).copy()
            
            # 随机生成一个附近的目标点 (偏移 ±10cm)
            target_t = np.eye(4)
            target_t[:3, :3] = current_mat # 保持姿态不变
            target_t[:3, 3] = current_pos + np.random.uniform(-0.1, 0.1, size=3)
            
            # 运行测试
            ik_q, dt, p_err, o_err = test_ik_performance(arm, target_t, data.qpos[:6])
            target_q = ik_q.ravel() 
            
            # 累加数据
            sum_dt += dt
            sum_p_err += p_err
            sum_o_err += o_err
            current_test += 1
            
            # 打印单次简报，监控进程
            print(f"[{current_test:02d}/{max_tests}] 采样完成 | 耗时: {dt:.4f} ms | 位置误差: {p_err * 1000:.4f} mm")

            # 达到样本量，输出统计报告并退出
            if current_test >= max_tests:
                avg_dt = sum_dt / max_tests
                avg_p_err = sum_p_err / max_tests
                avg_o_err = sum_o_err / max_tests
                
                print(f"\n" + "="*55)
                print(f" 📊 IK 算法综合性能评价报告 (样本量 N={max_tests})")
                print(f" 测试域: 局部空间收敛 (平移扰动 ±0.1m, 姿态恒定)")
                print(f"-------------------------------------------------------")
                print(f" 平均计算耗时 (Mean Computation Time): {avg_dt:.4f} ms")
                print(f" 平均位置误差 (Mean Position Error):   {avg_p_err * 1000:.4f} mm")
                print(f" 平均姿态误差 (Mean Orientation Error):{avg_o_err:.6f}")
                print("="*55)
                break # 结束仿真循环

        viewer.sync()
        # 保证仿真步长稳定
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))