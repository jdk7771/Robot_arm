#include <iostream>
#include <cstdio>
#include <mujoco/mujoco.h>
#include ""

int main() {
    char error[1000];
    // 确认路径指向的是 UR5e
    const char* model_path = "/home/jiang/tools/mujoco_menagerie/universal_robots_ur5e/ur5e.xml";

    mjModel* m = mj_loadXML(model_path, 0, error, 1000);
    if (!m) {
        std::printf("模型加载失败: %s\n", error);
        return 1;
    }
    mjData* d = mj_makeData(m);

    // 计算物理状态
    mj_step(m, d);

    // 【修改点 1】更改匹配名称：UR5e 的末端 Body 名称通常是 "wrist_3_link" 或 "flange"
    const char* ee_name = "wrist_3_link"; 
    int ee_id = mj_name2id(m, mjOBJ_BODY, ee_name);

    // 如果找不到 wrist_3_link，尝试找 attachment site 或 flange
    if (ee_id == -1) {
        ee_id = mj_name2id(m, mjOBJ_BODY, "flange");
    }

    std::printf("================ UR5e 状态输出 ================\n");
    
    // 【修改点 2】UR5e 只有 6 个关节，循环上限设为 6
    std::printf("[关节角度 qpos]:\n");
    for (int i = 0; i < m->nq; ++i) { // 使用 m->nq 自动获取模型关节总数，更安全
        std::printf("  Joint %d: %.4f rad\n", i + 1, d->qpos[i]);
    }

    // 输出末端位置
    if (ee_id != -1) {
        mjtNum* pos = d->xpos + 3 * ee_id;
        std::printf("\n[末端位置 xpos - 参照对象: %s]:\n", mj_id2name(m, mjOBJ_BODY, ee_id));
        std::printf("  X: %.4f m\n", pos[0]);
        std::printf("  Y: %.4f m\n", pos[1]);
        std::printf("  Z: %.4f m\n", pos[2]);
    } else {
        std::printf("\n错误: 无法在模型中定位末端执行器 (尝试了 wrist_3_link/flange)。\n");
    }
    std::printf("==============================================\n");

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}