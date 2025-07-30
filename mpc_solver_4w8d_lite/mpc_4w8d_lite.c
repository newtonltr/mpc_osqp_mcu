#include "mpc_4w8d_lite.h"
#include <string.h>
#include <math.h>

// === 实用函数 ===
#define CLAMP(x, min_val, max_val) ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

/**
 * @brief 初始化MPC控制器（严格按照文档参数）
 * 
 * 按照文档第3.2.2节的参数设置：
 * - N = 10 (预测时域)
 * - Ts = 0.01 (采样时间100Hz)
 * - Q = diag([10,10,10,10]) (状态权重)
 * - R = diag([0.1,0.1,0.1,0.1]) (输入权重)
 * - theta_max = 45°, omega_max = 90°/s
 */
int mpc_4w8d_init(mpc_4w8d_t *mpc) {
    if (!mpc) return -1;
    
    // 清零整个结构体
    memset(mpc, 0, sizeof(mpc_4w8d_t));
    
    // === 初始化基础权重矩阵（按文档示例值） ===
    // Q = diag([10.0, 10.0, 10.0, 10.0])
    for (int i = 0; i < MPC_NX; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            mpc->Q[i][j] = (i == j) ? DEFAULT_Q_WEIGHT : 0.0f;
        }
    }
    
    // R = diag([0.1, 0.1, 0.1, 0.1])
    for (int i = 0; i < MPC_NU; i++) {
        for (int j = 0; j < MPC_NU; j++) {
            mpc->R[i][j] = (i == j) ? DEFAULT_R_WEIGHT : 0.0f;
        }
    }
    
    // QN = Q (终端权重等于状态权重)
    for (int i = 0; i < MPC_NX; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            mpc->QN[i][j] = mpc->Q[i][j];
        }
    }
    
    // === 设置约束（按文档示例值） ===
    float theta_max_rad = DEG_TO_RAD(DEFAULT_THETA_MAX_DEG);
    float omega_max_rad = DEG_TO_RAD(DEFAULT_OMEGA_MAX_DEG);
    
    for (int i = 0; i < MPC_NX; i++) {
        mpc->theta_min[i] = -theta_max_rad;
        mpc->theta_max[i] = theta_max_rad;
    }
    
    for (int i = 0; i < MPC_NU; i++) {
        mpc->omega_min[i] = -omega_max_rad;
        mpc->omega_max[i] = omega_max_rad;
    }
    
    // === 设置优化参数 ===
    mpc->learning_rate = 0.1f;          // 梯度下降学习率
    mpc->max_iterations = 50;           // 最大迭代次数
    mpc->use_warm_start = true;         // 使用热启动
    
    // === 构建所有矩阵（按文档顺序） ===
    // 1. 构建状态空间矩阵 A = I_4, B = Ts * I_4
    mpc_4w8d_build_state_space_matrices(mpc);
    
    // 2. 构建预测矩阵 A_bar, B_bar
    mpc_4w8d_build_prediction_matrices(mpc);
    
    // 3. 构建堆叠权重矩阵 Q_bar, R_bar
    mpc_4w8d_build_weight_matrices(mpc);
    
    // 4. 构建标准QP形式矩阵 P, A_c
    mpc_4w8d_build_qp_matrices(mpc);
    
    mpc->initialized = true;
    return 0;
}

/**
 * @brief 构建状态空间矩阵A和B（按文档第1.3节）
 * 
 * 按照文档公式：
 * A = I_4 (4×4单位矩阵)
 * B = Ts * I_4 (采样时间乘以4×4单位矩阵)
 */
int mpc_4w8d_build_state_space_matrices(mpc_4w8d_t *mpc) {
    if (!mpc) return -1;
    
    // 构建状态转移矩阵 A = I_4
    for (int i = 0; i < MPC_NX; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            mpc->A[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 构建输入矩阵 B = Ts * I_4
    for (int i = 0; i < MPC_NX; i++) {
        for (int j = 0; j < MPC_NU; j++) {
            mpc->B[i][j] = (i == j) ? Ts : 0.0f;
        }
    }
    
    return 0;
}

/**
 * @brief 构建预测矩阵A_bar和B_bar（按文档第2.4节）
 * 
 * 按照文档公式：
 * A_bar = [A; A^2; A^3; ...; A^N]
 * 由于A = I_4，所以A_bar = [I; I; I; ...; I] (N×nx, nx)
 * 
 * B_bar为下三角块带状结构 (N×nx, N×nu)：
 * B_bar = [B    0    0    ...  0  ]
 *         [AB   B    0    ...  0  ]
 *         [A²B  AB   B    ...  0  ]
 *         [...  ...  ...  ...  ...]
 *         [A^(N-1)B ... AB B]
 */
int mpc_4w8d_build_prediction_matrices(mpc_4w8d_t *mpc) {
    if (!mpc) return -1;
    
    // 清零预测矩阵
    memset(mpc->A_bar, 0, sizeof(mpc->A_bar));
    memset(mpc->B_bar, 0, sizeof(mpc->B_bar));
    
    // === 构建A_bar：由于A = I，所以A^i = I for all i ===
    for (int i = 0; i < MPC_N; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            for (int k = 0; k < MPC_NX; k++) {
                int row = i * MPC_NX + j;
                mpc->A_bar[row][k] = (j == k) ? 1.0f : 0.0f;  // A^(i+1) = I
            }
        }
    }
    
    // === 构建B_bar：下三角块带状结构 ===
    // 按照文档：B_bar[i,j] = A^(i-j) * B if i >= j, else 0
    for (int i = 0; i < MPC_N; i++) {        // 预测步数
        for (int j = 0; j < MPC_N; j++) {    // 控制输入时刻
            if (i >= j) {
                // 计算 A^(i-j) * B
                // 由于A = I，所以A^(i-j) = I，因此A^(i-j) * B = B
                for (int p = 0; p < MPC_NX; p++) {
                    for (int q = 0; q < MPC_NU; q++) {
                        int row = i * MPC_NX + p;
                        int col = j * MPC_NU + q;
                        mpc->B_bar[row][col] = mpc->B[p][q];  // B = Ts * I
                    }
                }
            }
            // else: 上三角部分为0（已通过memset清零）
        }
    }
    
    return 0;
}

/**
 * @brief 构建堆叠权重矩阵Q_bar和R_bar（按文档数学模型）
 * 
 * 按照文档公式：
 * Q_bar = diag(Q, Q, ..., Q, QN) (N×nx, N×nx) - 前(N-1)个Q，最后一个QN
 * R_bar = diag(R, R, ..., R) (N×nu, N×nu) - 所有N个R
 */
int mpc_4w8d_build_weight_matrices(mpc_4w8d_t *mpc) {
    if (!mpc) return -1;
    
    // 清零权重矩阵
    memset(mpc->Q_bar, 0, sizeof(mpc->Q_bar));
    memset(mpc->R_bar, 0, sizeof(mpc->R_bar));
    
    // === 构建Q_bar：块对角矩阵 ===
    for (int k = 0; k < MPC_N; k++) {
        for (int i = 0; i < MPC_NX; i++) {
            for (int j = 0; j < MPC_NX; j++) {
                int row = k * MPC_NX + i;
                int col = k * MPC_NX + j;
                if (k == MPC_N - 1) {
                    // 最后一个块使用终端权重QN
                    mpc->Q_bar[row][col] = mpc->QN[i][j];
                } else {
                    // 前面的块使用状态权重Q
                    mpc->Q_bar[row][col] = mpc->Q[i][j];
                }
            }
        }
    }
    
    // === 构建R_bar：块对角矩阵 ===
    for (int k = 0; k < MPC_N; k++) {
        for (int i = 0; i < MPC_NU; i++) {
            for (int j = 0; j < MPC_NU; j++) {
                int row = k * MPC_NU + i;
                int col = k * MPC_NU + j;
                mpc->R_bar[row][col] = mpc->R[i][j];
            }
        }
    }
    
    return 0;
}

/**
 * @brief 构建标准QP形式的P矩阵和A_c矩阵（按文档第2.4节）
 * 
 * 按照文档公式：
 * P = B_bar^T * Q_bar * B_bar + R_bar (N×nu, N×nu)
 * A_c = [B_bar; I] (N×(nx+nu), N×nu) - 上半部分B_bar，下半部分单位矩阵I
 */
int mpc_4w8d_build_qp_matrices(mpc_4w8d_t *mpc) {
    if (!mpc) return -1;
    
    // 清零QP矩阵
    memset(mpc->P, 0, sizeof(mpc->P));
    memset(mpc->A_c, 0, sizeof(mpc->A_c));
    
    // === 构建P矩阵: P = B_bar^T * Q_bar * B_bar + R_bar ===
    // 步骤1: 计算 B_bar^T * Q_bar * B_bar
    for (int i = 0; i < MPC_N * MPC_NU; i++) {
        for (int j = 0; j < MPC_N * MPC_NU; j++) {
            float sum = 0.0f;
            for (int k = 0; k < MPC_N * MPC_NX; k++) {
                for (int l = 0; l < MPC_N * MPC_NX; l++) {
                    sum += mpc->B_bar[k][i] * mpc->Q_bar[k][l] * mpc->B_bar[l][j];
                }
            }
            mpc->P[i][j] = sum;
        }
    }
    
    // 步骤2: 添加R_bar项: P = B_bar^T * Q_bar * B_bar + R_bar
    for (int i = 0; i < MPC_N * MPC_NU; i++) {
        for (int j = 0; j < MPC_N * MPC_NU; j++) {
            mpc->P[i][j] += mpc->R_bar[i][j];
        }
    }
    
    // === 构建约束矩阵A_c = [B_bar; I] ===
    // 上半部分：B_bar (状态约束)
    for (int i = 0; i < MPC_N * MPC_NX; i++) {
        for (int j = 0; j < MPC_N * MPC_NU; j++) {
            mpc->A_c[i][j] = mpc->B_bar[i][j];
        }
    }
    
    // 下半部分：I (输入约束)
    int offset = MPC_N * MPC_NX;
    for (int i = 0; i < MPC_N * MPC_NU; i++) {
        mpc->A_c[offset + i][i] = 1.0f;
    }
    
    return 0;
}

/**
 * @brief 计算QP线性项q向量（按文档公式）
 * 
 * 按照文档公式：
 * q = B_bar^T * Q_bar * (A_bar * x_current - r_stacked)
 * 
 * 其中r_stacked是将参考值重复N次的堆叠向量
 */
int mpc_4w8d_calculate_q_vector(mpc_4w8d_t *mpc, const float x_current[MPC_NX], float q_vec[MPC_N * MPC_NU]) {
    if (!mpc || !x_current || !q_vec) return -1;
    
    // 步骤1: 计算 A_bar * x_current
    float Ax_current[MPC_N * MPC_NX];
    for (int i = 0; i < MPC_N * MPC_NX; i++) {
        Ax_current[i] = 0.0f;
        for (int j = 0; j < MPC_NX; j++) {
            Ax_current[i] += mpc->A_bar[i][j] * x_current[j];
        }
    }
    
    // 步骤2: 构建r_stacked（将参考值重复N次）
    float r_stacked[MPC_N * MPC_NX];
    for (int i = 0; i < MPC_N; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            r_stacked[i * MPC_NX + j] = mpc->theta_ref[j];
        }
    }
    
    // 步骤3: 计算误差向量 error_stacked = A_bar * x_current - r_stacked
    float error_stacked[MPC_N * MPC_NX];
    for (int i = 0; i < MPC_N * MPC_NX; i++) {
        error_stacked[i] = Ax_current[i] - r_stacked[i];
    }
    
    // 步骤4: 计算 q = B_bar^T * Q_bar * error_stacked
    for (int i = 0; i < MPC_N * MPC_NU; i++) {
        q_vec[i] = 0.0f;
        for (int j = 0; j < MPC_N * MPC_NX; j++) {
            for (int k = 0; k < MPC_N * MPC_NX; k++) {
                q_vec[i] += mpc->B_bar[j][i] * mpc->Q_bar[j][k] * error_stacked[k];
            }
        }
    }
    
    return 0;
}

/**
 * @brief 计算QP约束边界向量l,u（按文档公式）
 * 
 * 按照文档公式：
 * l = [x_min_stacked - A_bar * x_current; u_min_stacked]
 * u = [x_max_stacked - A_bar * x_current; u_max_stacked]
 * 
 * 约束向量维度：N*(nx+nu) = 10*8 = 80
 * 前40个元素：状态约束边界
 * 后40个元素：输入约束边界
 */
int mpc_4w8d_calculate_constraint_bounds(mpc_4w8d_t *mpc, const float x_current[MPC_NX]) {
    if (!mpc || !x_current) return -1;
    
    // 步骤1: 计算 A_bar * x_current
    float Ax_current[MPC_N * MPC_NX];
    for (int i = 0; i < MPC_N * MPC_NX; i++) {
        Ax_current[i] = 0.0f;
        for (int j = 0; j < MPC_NX; j++) {
            Ax_current[i] += mpc->A_bar[i][j] * x_current[j];
        }
    }
    
    // 步骤2: 设置状态约束边界
    // l[0:N*nx] = x_min_stacked - A_bar * x_current
    // u[0:N*nx] = x_max_stacked - A_bar * x_current
    for (int i = 0; i < MPC_N; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            int idx = i * MPC_NX + j;
            mpc->l_bounds[idx] = mpc->theta_min[j] - Ax_current[idx];
            mpc->u_bounds[idx] = mpc->theta_max[j] - Ax_current[idx];
        }
    }
    
    // 步骤3: 设置输入约束边界
    // l[N*nx:N*(nx+nu)] = u_min_stacked
    // u[N*nx:N*(nx+nu)] = u_max_stacked
    int offset = MPC_N * MPC_NX;
    for (int i = 0; i < MPC_N; i++) {
        for (int j = 0; j < MPC_NU; j++) {
            int idx = offset + i * MPC_NU + j;
            mpc->l_bounds[idx] = mpc->omega_min[j];
            mpc->u_bounds[idx] = mpc->omega_max[j];
        }
    }
    
    return 0;
}

/**
 * @brief 设置目标角度（度）
 */
int mpc_4w8d_set_target(mpc_4w8d_t *mpc, const float theta_ref_deg[MPC_NX]) {
    if (!mpc) return -1;
    
    // 转换度到弧度
    for (int i = 0; i < MPC_NX; i++) {
        mpc->theta_ref[i] = DEG_TO_RAD(theta_ref_deg[i]);
    }
    
    return 0;
}

/**
 * @brief 设置当前角度（从传感器读取后更新）
 */
int mpc_4w8d_set_current(mpc_4w8d_t *mpc, const float theta_current_deg[MPC_NX]) {
    if (!mpc) return -1;
    
    for (int i = 0; i < MPC_NX; i++) {
        mpc->x_current[i] = DEG_TO_RAD(theta_current_deg[i]);
    }
    
    return 0;
}

/**
 * @brief 设置权重矩阵（按文档格式）
 */
int mpc_4w8d_set_weights(mpc_4w8d_t *mpc, float q_weight, float r_weight) {
    if (!mpc) return -1;
    
    // 更新状态权重矩阵Q（对角矩阵）
    for (int i = 0; i < MPC_NX; i++) {
        mpc->Q[i][i] = q_weight;
    }
    
    // 更新输入权重矩阵R（对角矩阵）
    for (int i = 0; i < MPC_NU; i++) {
        mpc->R[i][i] = r_weight;
    }
    
    // 更新终端权重矩阵QN = Q
    for (int i = 0; i < MPC_NX; i++) {
        for (int j = 0; j < MPC_NX; j++) {
            mpc->QN[i][j] = mpc->Q[i][j];
        }
    }
    
    // 重新构建依赖的矩阵
    mpc_4w8d_build_weight_matrices(mpc);
    mpc_4w8d_build_qp_matrices(mpc);
    
    return 0;
}

/**
 * @brief 标准QP约束投影（按文档数学模型）
 * 
 * 使用标准QP约束形式：l ≤ A_c * z ≤ u
 * 由于A_c = [B_bar; I]，分两部分处理：
 * 1. 状态约束：l[0:N*nx] ≤ B_bar * z ≤ u[0:N*nx]
 * 2. 输入约束：l[N*nx:] ≤ z ≤ u[N*nx:]
 */
int mpc_4w8d_project_to_feasible_set(mpc_4w8d_t *mpc, float u_vec[MPC_N * MPC_NU]) {
    if (!mpc || !u_vec) return -1;
    
    // 计算 A_c * z = [B_bar * z; z]
    float Ac_z[MPC_N * (MPC_NX + MPC_NU)];
    
    // 上半部分：B_bar * z (状态约束)
    for (int i = 0; i < MPC_N * MPC_NX; i++) {
        Ac_z[i] = 0.0f;
        for (int j = 0; j < MPC_N * MPC_NU; j++) {
            Ac_z[i] += mpc->A_c[i][j] * u_vec[j];
        }
    }
    
    // 下半部分：z (输入约束)
    int offset = MPC_N * MPC_NX;
    for (int i = 0; i < MPC_N * MPC_NU; i++) {
        Ac_z[offset + i] = u_vec[i];
    }
    
    // 投影到约束边界：l ≤ A_c * z ≤ u
    // 这里使用简化的投影：直接对输入约束部分进行投影
    // 更复杂的状态约束投影需要QP求解器或更高级的投影算法
    
    // 对输入约束进行投影（下半部分）
    for (int i = 0; i < MPC_N * MPC_NU; i++) {
        int constraint_idx = offset + i;
        if (Ac_z[constraint_idx] < mpc->l_bounds[constraint_idx]) {
            u_vec[i] = mpc->l_bounds[constraint_idx];
        } else if (Ac_z[constraint_idx] > mpc->u_bounds[constraint_idx]) {
            u_vec[i] = mpc->u_bounds[constraint_idx];
        }
    }
    
    return 0;
}

/**
 * @brief MPC控制器求解（手写梯度下降）
 * 
 * 使用标准QP形式的梯度下降法：
 * 目标函数: min 1/2 * z^T * P * z + q^T * z
 * 梯度: grad = P * z + q
 * 更新: z = z - learning_rate * grad
 */
int mpc_4w8d_solve(mpc_4w8d_t *mpc, const float x_current_deg[MPC_NX], float u_optimal_deg[MPC_NU]) {
    if (!mpc || !x_current_deg || !u_optimal_deg || !mpc->initialized) return -1;
    
    // 转换当前状态为弧度
    float x_current_rad[MPC_NX];
    for (int i = 0; i < MPC_NX; i++) {
        x_current_rad[i] = DEG_TO_RAD(x_current_deg[i]);
    }
    
    // 初始化控制输入向量z（热启动或零初始化）
    float z[MPC_N * MPC_NU];
    if (mpc->use_warm_start) {
        // 使用上一次的解作为初始猜测
        for (int i = 0; i < MPC_N; i++) {
            for (int j = 0; j < MPC_NU; j++) {
                z[i * MPC_NU + j] = mpc->u_prev[i][j];
            }
        }
    } else {
        // 零初始化
        memset(z, 0, sizeof(z));
    }
    
    // ===============================================
    // 在线参数更新（按文档2.4节"对实时实现的关键启示"）
    // ===============================================
    
    // 1. 计算QP线性项q（在线更新）
    // q = B_bar^T * Q_bar * (A_bar * x_current - r_stacked)
    // 依赖当前状态x_current和参考值r_target，必须每个控制周期重新计算
    float q_vec[MPC_N * MPC_NU];
    mpc_4w8d_calculate_q_vector(mpc, x_current_rad, q_vec);
    
    // 2. 计算QP约束边界l,u（在线更新）
    // l = [x_min_stacked - A_bar * x_current; u_min_stacked]
    // u = [x_max_stacked - A_bar * x_current; u_max_stacked]
    // 依赖当前状态x_current，必须每个控制周期重新计算
    mpc_4w8d_calculate_constraint_bounds(mpc, x_current_rad);
    
    // 注意：P矩阵和A_c矩阵在初始化时已离线计算，不需要在线更新
    
    // ===============================================
    // 标准QP求解：min 1/2*z^T*P*z + q^T*z, s.t. l <= A_c*z <= u
    // ===============================================
    
    // 梯度下降迭代
    for (int iter = 0; iter < mpc->max_iterations; iter++) {
        // 计算梯度: grad = P * z + q
        float grad[MPC_N * MPC_NU];
        for (int i = 0; i < MPC_N * MPC_NU; i++) {
            grad[i] = q_vec[i];  // 线性项
            // 加上二次项 P * z
            for (int j = 0; j < MPC_N * MPC_NU; j++) {
                grad[i] += mpc->P[i][j] * z[j];
            }
        }
        
        // 梯度下降更新: z = z - learning_rate * grad
        for (int i = 0; i < MPC_N * MPC_NU; i++) {
            z[i] -= mpc->learning_rate * grad[i];
        }
        
        // 投影到可行域
        mpc_4w8d_project_to_feasible_set(mpc, z);
    }
    
    // 保存解用于下次热启动
    for (int i = 0; i < MPC_N; i++) {
        for (int j = 0; j < MPC_NU; j++) {
            mpc->u_prev[i][j] = z[i * MPC_NU + j];
        }
    }
    
    // 提取第一个时刻的控制输入作为最优解
    for (int i = 0; i < MPC_NU; i++) {
        mpc->u_optimal[i] = z[i];  // 弧度/秒
        u_optimal_deg[i] = RAD_TO_DEG(mpc->u_optimal[i]);  // 转换为度/秒
    }
    
    return 0;
} 