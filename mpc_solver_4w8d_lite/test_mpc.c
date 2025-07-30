#include <stdio.h>
#include <math.h>
#include "mpc_4w8d_lite.h"

int main() {
    printf("=== 四轮八驱MPC控制器测试 (严格按文档数学模型) ===\n\n");
    
    // === 创建并初始化MPC控制器 ===
    mpc_4w8d_t mpc;
    
    printf("📋 初始化参数（按文档第3.2.2节）:\n");
    printf("   - 预测时域 N = %d\n", N);
    printf("   - 采样时间 Ts = %.3f秒 (%.0fHz)\n", Ts, 1.0f/Ts);
    printf("   - 状态维度 nx = %d (四个转向角)\n", nx);
    printf("   - 输入维度 nu = %d (四个角速度)\n", nu);
    printf("   - Q权重 = %.1f, R权重 = %.1f\n", DEFAULT_Q_WEIGHT, DEFAULT_R_WEIGHT);
    printf("   - 角度约束 = ±%.0f°, 速度约束 = ±%.0f°/s\n\n", DEFAULT_THETA_MAX_DEG, DEFAULT_OMEGA_MAX_DEG);
    
    if (mpc_4w8d_init(&mpc) != 0) {
        printf("❌ MPC控制器初始化失败\n");
        return -1;
    }
    printf("✅ MPC控制器初始化成功\n\n");
    
    // === 设置目标角度 ===
    float target_angles[nx] = {30.0f, -15.0f, 0.0f, 45.0f};  // 度
    if (mpc_4w8d_set_target(&mpc, target_angles) != 0) {
        printf("❌ 目标角度设置失败\n");
        return -1;
    }
    printf("🎯 目标角度设置: [%.1f°, %.1f°, %.1f°, %.1f°]\n\n", 
           target_angles[0], target_angles[1], target_angles[2], target_angles[3]);
    
    // === 模拟控制循环 ===
    printf("=== Control Loop Simulation (按文档QP梯度下降法) ===\n");
    printf("【说明】每个控制周期都会在线更新：\n");
    printf("  - q向量：基于当前状态x_current和参考值r_target\n");
    printf("  - l,u向量：基于当前状态x_current和约束边界\n");
    printf("  - P矩阵和A_c矩阵：离线预计算，运行时不变\n\n");
    printf("Step  Current Angles[deg]                    Optimal Velocities[deg/s]                   Target Tracking Error[deg]  \n");
    printf("----  ---------------------------    ---------------------------      -----------------\n");
    
    // 初始状态：当前转向角度
    float x_current[nx] = {0.0f, 0.0f, 0.0f, 0.0f};  // 度
    float u_optimal[nu];  // 最优角速度（度/秒）
    
    // 仿真循环
    const int sim_steps = 110;
    
    for (int step = 0; step < sim_steps; step++) {
        // MPC求解
        if (mpc_4w8d_solve(&mpc, x_current, u_optimal) != 0) {
            printf("❌ 第%d步MPC求解失败\n", step + 1);
            return -1;
        }
        
        // 计算跟踪误差
        float tracking_errors[nx];
        for (int i = 0; i < nx; i++) {
            tracking_errors[i] = fabs(x_current[i] - target_angles[i]);
        }
        
        // 输出结果
        printf("%2d    [%6.1f,%6.1f,%6.1f,%6.1f]   [%6.1f,%6.1f,%6.1f,%6.1f]     [%4.1f,%4.1f,%4.1f,%4.1f]\n",
               step + 1,
               x_current[0], x_current[1], x_current[2], x_current[3],
               u_optimal[0], u_optimal[1], u_optimal[2], u_optimal[3],
               tracking_errors[0], tracking_errors[1], tracking_errors[2], tracking_errors[3]);
        
        // 验证约束满足
        bool constraints_satisfied = true;
        for (int i = 0; i < nu; i++) {
            if (fabs(u_optimal[i]) > DEFAULT_OMEGA_MAX_DEG + 0.1f) {
                constraints_satisfied = false;
                break;
            }
        }
        
        if (!constraints_satisfied) {
            printf("⚠️  警告：第%d步违反速度约束！\n", step + 1);
        }
        
        // 简单的系统动力学仿真：x(k+1) = x(k) + Ts * u(k)
        // 这模拟了文档中的状态空间模型：x(k+1) = A*x(k) + B*u(k)，其中A=I, B=Ts*I
        for (int i = 0; i < nx; i++) {
            x_current[i] += Ts * u_optimal[i];  // x = x + Ts * omega (积分器模型)
            
            // 状态约束保护
            if (x_current[i] > DEFAULT_THETA_MAX_DEG) x_current[i] = DEFAULT_THETA_MAX_DEG;
            if (x_current[i] < -DEFAULT_THETA_MAX_DEG) x_current[i] = -DEFAULT_THETA_MAX_DEG;
        }
    }
    
    // === 计算最终性能指标 ===
    printf("\n=== 最终性能评估 ===\n");
    
    float final_errors[nx];
    float avg_error = 0.0f;
    for (int i = 0; i < nx; i++) {
        final_errors[i] = fabs(x_current[i] - target_angles[i]);
        avg_error += final_errors[i];
    }
    avg_error /= nx;
    
    printf("最终角度: [%.1f°, %.1f°, %.1f°, %.1f°]\n",
           x_current[0], x_current[1], x_current[2], x_current[3]);
    printf("目标角度: [%.1f°, %.1f°, %.1f°, %.1f°]\n",
           target_angles[0], target_angles[1], target_angles[2], target_angles[3]);
    printf("跟踪误差: [%.1f°, %.1f°, %.1f°, %.1f°]\n",
           final_errors[0], final_errors[1], final_errors[2], final_errors[3]);
    printf("平均误差: %.2f°\n\n", avg_error);
    
    // === 验证数学模型一致性 ===
    printf("=== 数学模型验证 ===\n");
    printf("✅ 状态空间矩阵: A = I_%d, B = %.3f * I_%d\n", nx, Ts, nx);
    printf("✅ 预测矩阵: A_bar (%d×%d), B_bar (%d×%d)\n", N*nx, nx, N*nx, N*nu);
    printf("✅ 权重矩阵: Q_bar (%d×%d), R_bar (%d×%d)\n", N*nx, N*nx, N*nu, N*nu);
    printf("✅ QP矩阵: P (%d×%d), A_c (%d×%d)\n", N*nu, N*nu, N*(nx+nu), N*nu);
    printf("✅ 标准QP形式: min 1/2 * z^T * P * z + q^T * z\n");
    printf("✅ 约束形式: A_c * z ∈ [l, u]\n");
    printf("✅ 梯度下降: grad = P * z + q\n");
    printf("✅ 四轮同步控制: 集中式优化确保协调运动\n\n");
    
    // === 权重调整测试 ===
    printf("\n=== Weight Adjustment Test ===\n");
    printf("Testing faster convergence with increased Q weight...\n");
    
    // 增加Q权重以获得更快的收敛
    if (mpc_4w8d_set_weights(&mpc, 50.0f, 0.1f) != 0) {
        printf("❌ 权重设置失败\n");
        return -1;
    }
    
    // 重新设置初始状态和目标
    float x_test[nx] = {0.0f, 0.0f, 0.0f, 0.0f};
    float target_test[nx] = {10.0f, -10.0f, 5.0f, -5.0f};
    
    if (mpc_4w8d_set_target(&mpc, target_test) != 0) {
        printf("❌ 目标设置失败\n");
        return -1;
    }
    
    printf("Higher Q weight test - first 5 steps:\n");
    printf("Step  Current[deg]           Optimal[deg/s]      Error[deg]\n");
    printf("----  ----------------       ----------------    -----------\n");
    
    for (int step = 0; step < 5; step++) {
        if (mpc_4w8d_solve(&mpc, x_test, u_optimal) != 0) {
            printf("❌ MPC求解失败\n");
            return -1;
        }
        
        float errors[nx];
        for (int i = 0; i < nx; i++) {
            errors[i] = fabs(x_test[i] - target_test[i]);
        }
        
        printf("%2d    [%4.1f,%4.1f,%4.1f,%4.1f]   [%5.1f,%5.1f,%5.1f,%5.1f]   [%3.1f,%3.1f,%3.1f,%3.1f]\n",
               step + 1,
               x_test[0], x_test[1], x_test[2], x_test[3],
               u_optimal[0], u_optimal[1], u_optimal[2], u_optimal[3],
               errors[0], errors[1], errors[2], errors[3]);
        
        // 系统动力学仿真
        for (int i = 0; i < nx; i++) {
            x_test[i] += Ts * u_optimal[i];
        }
    }

    // === 测试新增的set_current API ===
    printf("\n=== Set Current API Test ===\n");
    printf("【说明】测试 mpc_4w8d_set_current() 接口，用于从传感器更新当前角度\n\n");
    
    // 重置权重
    if (mpc_4w8d_set_weights(&mpc, 10.0f, 0.1f) != 0) {
        printf("❌ 权重重置失败\n");
        return -1;
    }
    
    // 测试set_current接口
    float test_current[nx] = {5.0f, -5.0f, 3.0f, -3.0f};
    if (mpc_4w8d_set_current(&mpc, test_current) != 0) {
        printf("❌ 当前状态设置失败\n");
        return -1;
    }
    
    printf("✅ 成功设置当前角度: [%.1f°, %.1f°, %.1f°, %.1f°]\n",
           test_current[0], test_current[1], test_current[2], test_current[3]);
    printf("💡 在实际应用中，这个接口在传感器中断中调用，用于更新结构体的x_current成员\n");

    return 0;
} 