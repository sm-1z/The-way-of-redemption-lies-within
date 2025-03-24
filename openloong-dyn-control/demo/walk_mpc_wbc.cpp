/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "wbc_priority.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include <string>
#include <iostream>

#include "YKB.h"

const   double  dt = 0.001;
const   double  dt_200Hz = 0.005;
// MuJoCo load and compile model
char error[1000] = "Could not load binary model";

#if FIX == true
    mjModel* mj_model = mj_loadXML("../models/g1_29dof_with_hand_rev_1_0.xml", 0, error, 1000); // * XML YKB add
#elif FIX == false
    mjModel *mj_model = mj_loadXML("../models/scene.xml", 0, error, 1000);
#endif

mjData *mj_data = mj_makeData(mj_model);

int main(int argc, char **argv)
{
    // initialize classes
    UIctr uiController(mj_model, mj_data);        // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco

#if FIX == true
    Pin_KinDyn kinDynSolver("../models/g1_29dof_with_hand_rev_1_0.urdf"); // * URDF 2025.3.20 YKB add
#elif FIX == false
    Pin_KinDyn kinDynSolver("../models/AzureLoong.urdf"); // kinematics and dynamics solver
#endif

    DataBus RobotState(kinDynSolver.model_nv); // data bus
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    MPC MPC_solv(dt_200Hz);  // mpc controller
    GaitScheduler gaitScheduler(0.25, mj_model->opt.timestep); // gait scheduler

    #if FIX == true
        PVT_Ctr pvtCtr(mj_model->opt.timestep,"../common/g1_joint_ctrl_config.json");// * YKB
    #elif FIX == false
        PVT_Ctr pvtCtr(mj_model->opt.timestep,"../common/joint_ctrl_config.json");// PVT joint control
    #endif
    
    FootPlacement footPlacement; // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    DataLogger logger("../record/datalog.log"); // data logger


    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!

    // initialize variables

#if FIX == true
    double stand_legLength = 0.74; // * G1 YKB add
    double foot_height = 0.052; 
#elif FIX == false
    double stand_legLength = 1.01;//0.97;// desired baselink height
    double foot_height = 0.07; // distance between the foot ankel joint and the bottom
#endif
    
    double xv_des = 1.2;  // desired velocity in x direction
	int model_nv=kinDynSolver.model_nv;

    RobotState.width_hips = 0.229; // TODO: Need to Fix YKB

    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.03;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.15;
    footPlacement.legLength=stand_legLength;

    mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco

    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);

    // ini position and posture for foot-end and hand

    //stand_legLength：0.72 x 0.01 y 0.1145


#if FIX == true
    // * according to 1.RobotState.width_hips 2.stand_legLength YKB
    Eigen::Vector3d fe_l_pos_L_des={-0.01, 0.1145, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des={-0.01, -0.1145, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des={0.000, 0.000, 0.000};
    Eigen::Vector3d fe_r_eul_L_des={0.000, 0.000, 0.000};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des={0.03, 0.28, -0.08}; // x-前后 y-左右 z-上下
    Eigen::Vector3d hd_r_pos_L_des={0.03, -0.28, -0.08};
    Eigen::Vector3d hd_l_eul_L_des={1.57, 1.2, 1.57};
    Eigen::Vector3d hd_r_eul_L_des={-1.57, 1.2, -1.57};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));
#elif FIX == false
    Eigen::Vector3d fe_l_pos_L_des={-0.018, 0.113, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des={-0.018, -0.116, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des={-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des={0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des={-0.02, 0.32, -0.159};
    Eigen::Vector3d hd_r_pos_L_des={-0.02, -0.32, -0.159};
    Eigen::Vector3d hd_l_eul_L_des={-1.253, 0.122, -1.732};
    Eigen::Vector3d hd_r_eul_L_des={1.253, 0.122, 1.732};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));
#endif

    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1)= resLeg.jointPosRes + resHand.jointPosRes;
    WBC_solv.setQini(qIniDes,RobotState.q);

    // std::cout << "resLeg:" << resLeg.jointPosRes << std::endl;
    // std::cout << "resHand:" << resHand.jointPosRes << std::endl;
    std::cout << "qIniDes:" << qIniDes << std::endl;

    Eigen::VectorXd qTest = Eigen::VectorXd::Zero(mj_model->nq, 1);

    // register variable name for data logger
    logger.addIterm("simTime",1);
    logger.addIterm("motor_pos_des",model_nv - 6);
    logger.addIterm("motor_pos_cur",model_nv - 6);
    logger.addIterm("motor_vel_des",model_nv - 6);
    logger.addIterm("motor_vel_cur",model_nv - 6);
    logger.addIterm("motor_tor_des",model_nv - 6);
    logger.addIterm("rpyVal",3);
    logger.addIterm("base_omega_W",3);
    logger.addIterm("gpsVal",3);
    logger.addIterm("base_vel",3);
	logger.addIterm("dX_cal",12);
	logger.addIterm("Ufe",12);
	logger.addIterm("Xd",12);
	logger.addIterm("X_cur",12);
	logger.addIterm("X_cal",12);
    logger.finishItermAdding();

    //// -------------------------- main loop --------------------------------

    int  MPC_count = 0; // count for controlling the mpc running period

    double startHandSwingTime=3;
    double startSteppingTime=6;
    double startWalkingTime=9;
    double simEndTime=30; // 仿真时长

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    // while (!glfwWindowShouldClose(uiController.window)) {
    //     simstart = mj_data->time;
    //     while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) { // press "1" to pause and resume, "2" to step the simulation
    //         mj_step(mj_model, mj_data);
    //         simTime=mj_data->time;

    //     if (mj_data->time>=simEndTime)
    //         break;

    //     uiController.updateScene();
    //     }
    // }


    while (!glfwWindowShouldClose(uiController.window)) {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) { // press "1" to pause and resume, "2" to step the simulation
            // std::cout << "joint_qpos:" << RobotState.q << std::endl;
            mj_step(mj_model, mj_data);
            simTime=mj_data->time;
            // Read the sensors:
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            // joint number: arm-l: 0-6, arm-r: 7-13, head: 14, waist: 15-17, leg-l: 18-23, leg-r: 24-29
            // leg-l: 0-5, leg-r: 6-11, waist: 12-14, arm-l: 15-22, hand-l: 23-29, arm-r: 30-36, hand-r: 37-43

            if (simTime > startWalkingTime) {
                jsInterp.setWzDesLPara(0, 1);
                jsInterp.setVxDesLPara(xv_des, 2.0); // jsInterp.setVxDesLPara(0.9,1);
				RobotState.motionState = DataBus::Walk; // start walking
            } else
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            jsInterp.step();
            RobotState.js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
            jsInterp.dataBusWrite(RobotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

            if (simTime >= startSteppingTime) {
                // gait scheduler
                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);

                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);
            }
            // MPC_solv.enable(); // * YKB MPC_solv.cal()的使能，
            // ------------- MPC ------------
			// MPC_count = MPC_count + 1;
            // if (MPC_count > (dt_200Hz / dt-1)) {
            //     MPC_solv.dataBusRead(RobotState);
            //     MPC_solv.cal();
            //     MPC_solv.dataBusWrite(RobotState);
            //     MPC_count = 0;
            // }

            // ------------- WBC ------------
            // WBC Calculation
            // WBC_solv.dataBusRead(RobotState);
            // WBC_solv.computeDdq(kinDynSolver);
            // WBC_solv.computeTau();
            // WBC_solv.dataBusWrite(RobotState);
            // get the final joint command
            if (simTime <= startHandSwingTime) {
                // RobotState.motors_pos_des = eigen2std(qTest);
                RobotState.motors_pos_des = eigen2std(resHand.jointPosRes);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
            }
            else if (simTime <= startSteppingTime) {
                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
            }
            else {
                // MPC_solv.enable();
                // Eigen::Matrix<double, 1, nx>  L_diag;
                // Eigen::Matrix<double, 1, nu>  K_diag;
                // L_diag <<
                //        1.0, 1.0, 1.0,//eul
                //         1.0, 200.0,  1.0,//pCoM
                //         1e-7, 1e-7, 1e-7,//w
                //         100.0, 10.0, 1.0;//vCoM
                // K_diag <<
                //        1.0, 1.0, 1.0,//fl
                //         1.0, 1.0, 1.0,
                //         1.0, 1.0, 1.0,//fr
                //         1.0, 1.0, 1.0,1.0;
                // MPC_solv.set_weight(1e-6, L_diag, K_diag);

                // Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                // RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                // RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                // RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            }

            // joint PVT controller
            pvtCtr.dataBusRead(RobotState);
            if (simTime <= startHandSwingTime) { // TODO 3 可能和仿真时间有关
                pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415); // 单个仿真周期的角度变化
            }
            else if(simTime <= startSteppingTime)
            {
                pvtCtr.calMotorsPVT(200.0 / 1000.0 / 180.0 * 3.1415);
            } 
            else {
#if FIX == true // * YKB
                // pvtCtr.setJointPD(100,2,"J_left_ankle_pitch_joint");
                // pvtCtr.setJointPD(100,2,"J_left_ankle_roll_joint");
                // pvtCtr.setJointPD(100,2,"J_right_ankle_pitch_joint");
                // pvtCtr.setJointPD(100,2,"J_right_ankle_roll_joint");
                // pvtCtr.setJointPD(100,2,"J_left_knee_joint");
                // pvtCtr.setJointPD(100,2,"J_right_knee_joint");
#elif FIX == false
                pvtCtr.setJointPD(100,10,"J_ankle_l_pitch");
                pvtCtr.setJointPD(100,10,"J_ankle_l_roll");
                pvtCtr.setJointPD(100,10,"J_ankle_r_pitch");
                pvtCtr.setJointPD(100,10,"J_ankle_r_roll");
                pvtCtr.setJointPD(1000,100,"J_knee_l_pitch");
                pvtCtr.setJointPD(1000,100,"J_knee_r_pitch");
#endif
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            // give the joint torque command to Webots
            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            // print info to the console
//            printf("f_L=[%.3f, %.3f, %.3f]\n", RobotState.fL[0], RobotState.fL[1], RobotState.fL[2]);
//            printf("f_R=[%.3f, %.3f, %.3f]\n", RobotState.fR[0], RobotState.fR[1], RobotState.fR[2]);
//
//            printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
//            printf("basePos=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);

            // data save
            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motor_pos_des", RobotState.motors_pos_des);
            logger.recItermData("motor_pos_cur", RobotState.motors_pos_cur);
            logger.recItermData("motor_vel_des", RobotState.motors_vel_des);
            logger.recItermData("motor_vel_cur", RobotState.motors_vel_cur);
            logger.recItermData("motor_tor_des", RobotState.motors_tor_des);
            logger.recItermData("rpyVal", RobotState.rpy);
            logger.recItermData("base_omega_W", RobotState.base_omega_W);
            logger.recItermData("gpsVal", RobotState.basePos);
            logger.recItermData("base_vel", RobotState.dq.block<3, 1>(0, 0));
			logger.recItermData("dX_cal",RobotState.dX_cal);
			logger.recItermData("Ufe",RobotState.Fr_ff);
			logger.recItermData("Xd",RobotState.Xd);
			logger.recItermData("X_cur",RobotState.X_cur);
			logger.recItermData("X_cal",RobotState.X_cal);
			logger.finishLine();
        }

        if (mj_data->time>=simEndTime)
            break;

        uiController.updateScene();
    };
    // free visualization storage
    uiController.Close();

    return 0;
}
