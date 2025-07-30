/*
 * MPC���߼��㺯��ʵ��
 * ���ļ���osqp_genarate.py�Զ�����
 * ʵ��STM32��q, l, u������ʵʱ����
 */

#include "mpc_matrices.h"
#include <string.h>

void calculate_q_vector(OSQPFloat* q_new, const OSQPFloat* x_current, const OSQPFloat* r_target) {
    // ���� q = B_bar^T * Q_bar * (A_bar * x_current - r_stacked)
    
    // ����1: ����A_bar * x_current
    OSQPFloat Ax[MPC_N * MPC_NX];
    for (OSQPInt i = 0; i < MPC_N * MPC_NX; i++) {
        Ax[i] = 0.0f;
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            Ax[i] += A_bar_data[i * MPC_NX + j] * x_current[j];
        }
    }
    
    // ����2: ����A_bar * x_current - r_stacked
    OSQPFloat error_stacked[MPC_N * MPC_NX];
    for (OSQPInt i = 0; i < MPC_N; i++) {
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            error_stacked[i * MPC_NX + j] = Ax[i * MPC_NX + j] - r_target[j];
        }
    }
    
    // ����3: ����q = BT_Q * error_stacked
    for (OSQPInt i = 0; i < MPC_N * MPC_NU; i++) {
        q_new[i] = 0.0f;
        for (OSQPInt j = 0; j < MPC_N * MPC_NX; j++) {
            q_new[i] += BT_Q_data[i * MPC_N * MPC_NX + j] * error_stacked[j];
        }
    }
}

void calculate_constraint_bounds(OSQPFloat* l_new, OSQPFloat* u_new, const OSQPFloat* x_current) {
    // ����Լ���߽�
    // l = [x_min_stacked - A_bar * x_current; u_min_stacked]
    // u = [x_max_stacked - A_bar * x_current; u_max_stacked]
    
    // ����1: ����A_bar * x_current (��������Ĵ����߼�)
    OSQPFloat Ax[MPC_N * MPC_NX];
    for (OSQPInt i = 0; i < MPC_N * MPC_NX; i++) {
        Ax[i] = 0.0f;
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            Ax[i] += A_bar_data[i * MPC_NX + j] * x_current[j];
        }
    }
    
    // ����2: ����״̬Լ���߽�
    for (OSQPInt i = 0; i < MPC_N; i++) {
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            OSQPInt idx = i * MPC_NX + j;
            l_new[idx] = x_min_single[j] - Ax[idx];
            u_new[idx] = x_max_single[j] - Ax[idx];
        }
    }
    
    // ����3: ��������Լ���߽�
    for (OSQPInt i = 0; i < MPC_N; i++) {
        for (OSQPInt j = 0; j < MPC_NU; j++) {
            OSQPInt idx = MPC_N * MPC_NX + i * MPC_NU + j;
            l_new[idx] = u_min_single[j];
            u_new[idx] = u_max_single[j];
        }
    }
}
