/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
//
// Created by boxing on 23-12-29.
//
#pragma once

#include "qpOASES.hpp"
#include <algorithm>
#include <Eigen/Dense>
#include "data_bus.h"
#include "useful_math.h"
#include "priority_tasks.h"
#include "pino_kin_dyn.h"
#include <iostream>
#include <iomanip>

#include "YKB.h"

class WBC_priority {
public:
    int model_nv; // size of the system generalized coordinate dq
    Eigen::Vector3d tau_upp_stand_L, tau_low_stand_L; // foot end contact torque limit for stand state, in body frame
    Eigen::Vector3d tau_upp_walk_L, tau_low_walk_L;  // foot end contact torque limit for walk state, in body frame
    double f_z_low{0},f_z_upp{0};
    DataBus::LegState legStateCur;
    DataBus::MotionState motionStateCur;
    WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt);
    double miu{0.5};
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non; // dyn_Non= c*dq+g
    Eigen::MatrixXd Jc, dJc, Jfe, dJfe, Jfe_L, Jfe_R;
    Eigen::MatrixXd J_hd_l, J_hd_r, dJ_hd_l, dJ_hd_r;
    Eigen::MatrixXd Jsw, dJsw;
    Eigen::Matrix3d fe_rot_sw_W;
    Eigen::Vector3d fe_pos_sw_W;
    Eigen::Vector3d hd_l_pos_cur_W, hd_r_pos_cur_W;
    Eigen::Matrix3d hd_l_rot_cur_W, hd_r_rot_cur_W;
    Eigen::Vector3d fe_l_pos_des_W, fe_r_pos_des_W;
    Eigen::Matrix3d fe_l_rot_des_W, fe_r_rot_des_W;
    Eigen::Vector3d fe_l_pos_cur_W, fe_r_pos_cur_W;
    Eigen::Matrix3d fe_l_rot_cur_W, fe_r_rot_cur_W;
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd Fr_ff;  // 12*1, [fe_L, fe_R]
    Eigen::VectorXd delta_ddq;
    Eigen::VectorXd delta_Fr;
    Eigen::VectorXd eigen_xOpt;
    Eigen::VectorXd eigen_ddq_Opt;
    Eigen::VectorXd eigen_fr_Opt, eigen_tau_Opt;
    Eigen::MatrixXd Q1;
    Eigen::MatrixXd Q2;
    Eigen::VectorXd delta_q_final_kin, dq_final_kin, ddq_final_kin, tauJointRes;
    Eigen::Matrix3d fe_L_rot_L_off, fe_R_rot_L_off; // foot-end R w.r.t to the body frame in offset posture
    double l_shoulder_pitch = 0; //q(28) - qIniDes(28);
    double r_shoulder_pitch = 0; //q(34) - qIniDes(34);
    Eigen::Vector3d pCoMDes, pCoMCur;

    PriorityTasks kin_tasks_walk, kin_tasks_stand;
    void setQini(const Eigen::VectorXd &qIniDes, const Eigen::VectorXd &qIniCur);
    void computeTau();
    void dataBusRead(const DataBus &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeDdq(Pin_KinDyn &pinKinDynIn);
private:
    double timeStep{0.001};
    qpOASES::QProblem QP_prob;
    Eigen::MatrixXd Sf; // floating-base dynamics selection matrix
    Eigen::MatrixXd St_qpV1, St_qpV2; // state selection matrix

    qpOASES::int_t nWSR=100, last_nWSR{0};
    qpOASES::real_t cpu_time=0.1, last_cpu_time{0};
    int qpStatus{0};
    int QP_nv;
    int QP_nc;
    void copy_Eigen_to_real_t(qpOASES::real_t* target, const Eigen::MatrixXd &source, int nRows, int nCols);
    Eigen::MatrixXd J_base, dJ_base, Jcom;
    Eigen::MatrixXd J_hip_link;
    Eigen::Vector3d base_pos_des, base_pos, base_rpy_des, base_rpy_cur, hip_link_pos;
    Eigen::Matrix3d hip_link_rot, base_rot;
    Eigen::VectorXd swing_fe_pos_des_W, swing_fe_rpy_des_W;
    Eigen::Vector3d stance_fe_pos_cur_W;
    Eigen::Matrix3d stance_fe_rot_cur_W;
    Eigen::Vector3d stanceDesPos_W;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;
    Eigen::VectorXd qIniDes, qIniCur;

    static const int QP_nv_des=18;
    static const int QP_nc_des=22;

    qpOASES::real_t qp_H[QP_nv_des*QP_nv_des];
    qpOASES::real_t qp_A[QP_nc_des*QP_nv_des];
    qpOASES::real_t qp_g[QP_nv_des];
    qpOASES::real_t qp_lbA[QP_nc_des];
    qpOASES::real_t qp_ubA[QP_nc_des];
    qpOASES::real_t xOpt_iniGuess[QP_nv_des];
};

#if FIX == true // * YKB 用于q()索引，前7个为位置和四元数值 G1 q.len=50 青龙 q.len=38

#define Joint_hip_pitch_l 7
#define Joint_hip_roll_l 8
#define Joint_hip_yaw_l 9
#define Joint_knee_pitch_l 10
#define Joint_ankle_pitch_l 11
#define Joint_ankle_roll_l 12

#define Joint_hip_pitch_r 13
#define Joint_hip_roll_r 14
#define Joint_hip_yaw_r 15
#define Joint_knee_pitch_r 16
#define Joint_ankle_pitch_r 17
#define Joint_ankle_roll_r 18

#define Joint_waist_yaw 19
#define Joint_waist_roll 20
#define Joint_waist_pitch 21

#define Joint_shouder_pitch_l 22
#define Joint_shouder_roll_l 23
#define Joint_shouder_yaw_l 24
#define Joint_elbow_l 25
#define Joint_wrist_roll_l 26
#define Joint_wrist_pitch_l 27
#define Joint_wrist_yaw_l 28

#define Joint_shouder_pitch_r 36
#define Joint_shouder_roll_r 37
#define Joint_shouder_yaw_r 38
#define Joint_elbow_r 39
#define Joint_wrist_roll_r 40
#define Joint_wrist_pitch_r 41
#define Joint_wrist_yaw_r 42

#define Joint_head_yaw 50
#define Joint_head_pitch 50

#elif FIX == false

#define Joint_shouder_pitch_l 7
#define Joint_shouder_roll_l 8
#define Joint_shouder_yaw_l 9
#define Joint_elbow_l 10
#define Joint_wrist_roll_l 11
#define Joint_wrist_pitch_l 12
#define Joint_wrist_yaw_l 13

#define Joint_shouder_pitch_r 14
#define Joint_shouder_roll_r 15
#define Joint_shouder_yaw_r 16
#define Joint_elbow_r 17
#define Joint_wrist_roll_r 18
#define Joint_wrist_pitch_r 19
#define Joint_wrist_yaw_r 20

#define Joint_head_yaw 21
#define Joint_head_pitch 22

#define Joint_waist_pitch 23
#define Joint_waist_roll 24
#define Joint_waist_yaw 25

#define Joint_hip_roll_l 26
#define Joint_hip_yaw_l 27
#define Joint_hip_pitch_l 28
#define Joint_knee_pitch_l 29
#define Joint_ankle_pitch_l 30
#define Joint_ankle_roll_l 31

#define Joint_hip_roll_r 32
#define Joint_hip_yaw_r 33
#define Joint_hip_pitch_r 34
#define Joint_knee_pitch_r 35
#define Joint_ankle_pitch_r 36
#define Joint_ankle_roll_r 37

#endif


