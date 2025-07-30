#ifndef MPC_4W8D_LITE_H
#define MPC_4W8D_LITE_H

#include <stdint.h>
#include <stdbool.h>

// === MPC参数定义（严格按照文档第3.2.2节） ===
#define MPC_N  10                   // 预测时域 (文档中的N = 10)
#define MPC_NX 4                    // 状态维度 (文档中的nx) - 4个转向角
#define MPC_NU 4                    // 输入维度 (文档中的nu) - 4个角速度

// === 系统参数（按文档定义） ===
#define MPC_TS 0.01f                // 采样时间 (文档中的Ts) - 100Hz

// === 默认权重值 ===
#define DEFAULT_Q_WEIGHT 10.0f  // 状态权重（文档示例：Q = diag([10,10,10,10])）
#define DEFAULT_R_WEIGHT 0.1f   // 输入权重（文档示例：R = diag([0.1,0.1,0.1,0.1])）

// === 默认约束（按文档示例） ===
#define DEFAULT_THETA_MAX_DEG 45.0f  // 最大转向角（文档：theta_max = 45度）
#define DEFAULT_OMEGA_MAX_DEG 90.0f  // 最大角速度（文档：omega_max = 90度/秒）

// === 实用宏 ===
#define DEG_TO_RAD(deg) ((deg) * 3.14159265359f / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / 3.14159265359f)

/**
 * @brief 四轮八驱MPC控制器数据结构（严格按文档数学模型）
 * 
 * 变量命名完全遵循文档第3.2.2节的Python实现：
 * - N: 预测时域
 * - nx: 状态维度  
 * - nu: 输入维度
 * - Ts: 采样时间
 * - A: 状态转移矩阵 (nx×nx单位矩阵)
 * - B: 输入矩阵 (Ts×nx单位矩阵)
 * - A_bar: 预测状态转移矩阵 (N×nx, nx)
 * - B_bar: 预测输入矩阵 (N×nx, N×nu)
 * - Q_bar: 堆叠状态权重矩阵 (N×nx, N×nx)
 * - R_bar: 堆叠输入权重矩阵 (N×nu, N×nu)
 * - QN: 终端权重矩阵 (等于Q)
 */
typedef struct {
    // === 当前状态和控制输入 ===
    float x_current[MPC_NX];        // 当前状态：四个转向轮角度 [θ1, θ2, θ3, θ4]
    float u_optimal[MPC_NU];        // 最优控制输入：四个角速度 [ω1, ω2, ω3, ω4]
    float theta_ref[MPC_NX];        // 参考角度 [θ1_ref, θ2_ref, θ3_ref, θ4_ref]
    
    // === 基础状态空间矩阵（按文档第1.3节） ===
    float A[MPC_NX][MPC_NX];            // 状态转移矩阵 A = I_4 (4×4单位矩阵)
    float B[MPC_NX][MPC_NU];            // 输入矩阵 B = Ts * I_4 (Ts乘以4×4单位矩阵)
    
    // === 预测矩阵（按文档第2.4节构建） ===
    float A_bar[MPC_N * MPC_NX][MPC_NX];                    // 预测状态转移矩阵 A_bar (20×4)
    float B_bar[MPC_N * MPC_NX][MPC_N * MPC_NU];                // 预测输入矩阵 B_bar (20×20)
    
    // === 权重矩阵（按文档定义） ===
    float Q[MPC_NX][MPC_NX];            // 状态权重矩阵 Q
    float R[MPC_NU][MPC_NU];            // 输入权重矩阵 R  
    float QN[MPC_NX][MPC_NX];           // 终端权重矩阵 QN = Q
    
    // === 堆叠权重矩阵（按文档数学模型） ===
    float Q_bar[MPC_N * MPC_NX][MPC_N * MPC_NX];                // 堆叠状态权重矩阵 Q_bar (20×20)
    float R_bar[MPC_N * MPC_NU][MPC_N * MPC_NU];                // 堆叠输入权重矩阵 R_bar (20×20)
    
    // === 标准QP形式矩阵（按文档第2.4节） ===
    float P[MPC_N * MPC_NU][MPC_N * MPC_NU];                    // 二次型矩阵 P = B_bar^T * Q_bar * B_bar + R_bar (20×20)
    float A_c[MPC_N * (MPC_NX + MPC_NU)][MPC_N * MPC_NU];           // 约束矩阵 A_c = [B_bar; I] (40×20)
    
    // === 约束边界向量（按文档在线计算） ===
    float l_bounds[MPC_N * (MPC_NX + MPC_NU)];              // 约束下界 l = [x_min_stacked - A_bar*x; u_min_stacked] (40)
    float u_bounds[MPC_N * (MPC_NX + MPC_NU)];              // 约束上界 u = [x_max_stacked - A_bar*x; u_max_stacked] (40)
    
    // === 约束参数 ===
    float theta_min[MPC_NX];        // 最小转向角
    float theta_max[MPC_NX];        // 最大转向角
    float omega_min[MPC_NU];        // 最小角速度
    float omega_max[MPC_NU];        // 最大角速度
    
    // === 优化参数 ===
    float learning_rate;        // 梯度下降学习率
    int max_iterations;         // 最大迭代次数
    bool use_warm_start;        // 是否使用热启动
    
    // === 内部状态 ===
    float u_prev[MPC_N][MPC_NU];        // 上一次的控制序列（用于热启动）
    bool initialized;           // 初始化标志
    
} mpc_4w8d_t;

// === 核心接口函数 ===

/**
 * @brief 初始化MPC控制器（按文档参数）
 * @param mpc MPC控制器指针
 * @return 0: 成功, -1: 失败
 */
int mpc_4w8d_init(mpc_4w8d_t *mpc);

/**
 * @brief 设置目标角度
 * @param mpc MPC控制器指针
 * @param theta_ref_deg 目标角度数组（度）
 * @return 0: 成功, -1: 失败
 */
int mpc_4w8d_set_target(mpc_4w8d_t *mpc, const float theta_ref_deg[MPC_NX]);

/**
 * @brief 设置当前角度（从传感器读取后更新）
 * @param mpc MPC控制器指针
 * @param theta_current_deg 当前角度数组（度）
 * @return 0: 成功, -1: 失败
 */
int mpc_4w8d_set_current(mpc_4w8d_t *mpc, const float theta_current_deg[MPC_NX]);

/**
 * @brief 设置权重矩阵（按文档格式）
 * @param mpc MPC控制器指针
 * @param q_weight Q矩阵对角线权重值
 * @param r_weight R矩阵对角线权重值
 * @return 0: 成功, -1: 失败
 */
int mpc_4w8d_set_weights(mpc_4w8d_t *mpc, float q_weight, float r_weight);

/**
 * @brief MPC控制器求解（手写梯度下降）
 * @param mpc MPC控制器指针
 * @param x_current_deg 当前状态（度）
 * @param u_optimal_deg 输出最优控制输入（度/秒）
 * @return 0: 成功, -1: 失败
 */
int mpc_4w8d_solve(mpc_4w8d_t *mpc, const float x_current_deg[MPC_NX], float u_optimal_deg[MPC_NU]);

// === 内部构建函数（按文档数学模型） ===

/**
 * @brief 构建状态空间矩阵A和B（按文档第1.3节）
 * A = I_4, B = Ts * I_4
 */
int mpc_4w8d_build_state_space_matrices(mpc_4w8d_t *mpc);

/**
 * @brief 构建预测矩阵A_bar和B_bar（按文档第2.4节）
 * A_bar = [A; A^2; ...; A^N], B_bar为下三角块带状结构
 */
int mpc_4w8d_build_prediction_matrices(mpc_4w8d_t *mpc);

/**
 * @brief 构建堆叠权重矩阵Q_bar和R_bar（按文档数学模型）
 * Q_bar = diag(Q, Q, ..., Q, QN), R_bar = diag(R, R, ..., R)
 */
int mpc_4w8d_build_weight_matrices(mpc_4w8d_t *mpc);

/**
 * @brief 构建标准QP形式的P矩阵和A_c矩阵（按文档第2.4节）
 * P = B_bar^T * Q_bar * B_bar + R_bar, A_c = [B_bar; I]
 */
int mpc_4w8d_build_qp_matrices(mpc_4w8d_t *mpc);

/**
 * @brief 计算QP线性项q向量（按文档公式）
 * q = B_bar^T * Q_bar * (A_bar * x_current - r_stacked)
 */
int mpc_4w8d_calculate_q_vector(mpc_4w8d_t *mpc, const float x_current[MPC_NX], float q_vec[MPC_N * MPC_NU]);

/**
 * @brief 计算QP约束边界向量l,u（按文档公式）
 * l = [x_min_stacked - A_bar * x_current; u_min_stacked]
 * u = [x_max_stacked - A_bar * x_current; u_max_stacked]
 */
int mpc_4w8d_calculate_constraint_bounds(mpc_4w8d_t *mpc, const float x_current[MPC_NX]);

/**
 * @brief 约束投影（确保满足物理约束）
 * @param mpc MPC控制器指针
 * @param u_vec 控制输入序列
 * @return 0: 成功, -1: 失败
 */
int mpc_4w8d_project_to_feasible_set(mpc_4w8d_t *mpc, float u_vec[MPC_N * MPC_NU]);

#endif // MPC_4W8D_LITE_H 