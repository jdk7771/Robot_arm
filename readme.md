##启动Frank仿真器

# 1. 进入 MuJoCo 的二进制程序目录
cd ~/tools/mujoco/bin

# 2. 赋予执行权限（如果之前没做过）
chmod +x simulate

# 3. 启动 Franka 仿真
./simulate ~/tools/mujoco_menagerie/franka_emika_panda/panda.xml

# 3. 启动 ur5e 仿真
./simulate ~/tools/mujoco_menagerie/universal_robots_ur5e/ur5e.xml

