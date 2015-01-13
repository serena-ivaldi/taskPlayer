/*
 * Copyright (C) 2011-2012 MACSi Project
 * Author: Serena Ivaldi
 * email:  serena.ivaldi@isir.upmc.fr
 * website: www.macsi.isir.upmc.fr
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef TRAJPLAYER_H
#define TRAJPLAYER_H

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <macsi/modHelp/modHelp.h>
#include <macsi/objects/objects.h>


#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <deque>
#include <sstream>
#include <string>
#include <stdio.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define CONTROLMODE_VEL         0
#define CONTROLMODE_VELIMP_FIX  1
#define CONTROLMODE_VELIMP_VAR  2

#define INIT_MODE_CARTESIAN 0
#define INIT_MODE_CARTESIAN_JOINTS 1
#define INIT_MODE_RELATIVE 2

#define IMPSOFT 0
#define IMPMEDIUM 1
#define IMPHARD 2


#define STATUS_PLAYING  0
#define STATUS_BLOCKED  1

#define POSITION_THRESHOLD  5.0
#define WAIT_N_INITPOSE     20
#define REF_SPEED           10.0

 // for each matrix (i,j) i = iteration; j = data
struct trajectory_matrix
{
    yarp::sig::Matrix x_RA;
    yarp::sig::Matrix x_LA;
    yarp::sig::Matrix o_RA;
    yarp::sig::Matrix o_LA;
    yarp::sig::Matrix q_RA;
    yarp::sig::Matrix q_LA;
    yarp::sig::Matrix q_TO;

    yarp::sig::Matrix xd_RA;
    yarp::sig::Matrix xd_LA;
    yarp::sig::Matrix od_RA;
    yarp::sig::Matrix od_LA;
    yarp::sig::Matrix qd_RA;
    yarp::sig::Matrix qd_LA;
    yarp::sig::Matrix qd_TO;

    yarp::sig::Matrix timestamps;
};





class TrajPlayer: public yarp::os::RateThread
{
protected:

    // interface with icub
    // generic
    // yarp::dev::IPositionControl     *ipos;
    // //yarp::dev::ITorqueControl       *itrq;
    // yarp::dev::IImpedanceControl    *iimp;
    // yarp::dev::IControlMode         *icmd;
    // yarp::dev::IEncoders            *ienc;
    // yarp::dev::IVelocityControl     *ivel;
    // yarp::dev::ICartesianControl    *icrt;
    // yarp::dev::PolyDriver           *dd;
    // yarp::dev::PolyDriver           *ddCart;

    // left arm
    yarp::dev::IPositionControl     *ipos_L;
    //yarp::dev::ITorqueControl       *itrq_L;
    yarp::dev::IImpedanceControl    *iimp_L;
    yarp::dev::IControlMode         *icmd_L;
    yarp::dev::IEncoders            *ienc_L;
    yarp::dev::IVelocityControl     *ivel_L;
    yarp::dev::ICartesianControl    *icrt_L;
    yarp::dev::PolyDriver           *dd_L;
    yarp::dev::PolyDriver           *ddCart_L;
    // right arm
    yarp::dev::IPositionControl     *ipos_R;
    //yarp::dev::ITorqueControl       *itrq_R;
    yarp::dev::IImpedanceControl    *iimp_R;
    yarp::dev::IControlMode         *icmd_R;
    yarp::dev::IEncoders            *ienc_R;
    yarp::dev::IVelocityControl     *ivel_R;
    yarp::dev::ICartesianControl    *icrt_R;
    yarp::dev::PolyDriver           *dd_R;
    yarp::dev::PolyDriver           *ddCart_R;

    // torso
    yarp::dev::IPositionControl     *ipos_T;
    //yarp::dev::ITorqueControl       *itrq_T;
    yarp::dev::IImpedanceControl    *iimp_T;
    yarp::dev::IControlMode         *icmd_T;
    yarp::dev::IEncoders            *ienc_T;
    yarp::dev::IVelocityControl     *ivel_T;
    yarp::dev::PolyDriver           *dd_T;

    // for playing trajectory
    int njointsImpedance;

    // total trajectory
    // generic
    // yarp::sig::Matrix               traj_v; // joint velocity
    // yarp::sig::Matrix               traj_s; // joint stiffness
    // yarp::sig::Matrix               traj_d; // joint damping
    // yarp::sig::Vector               init_q; // initial joint configuration
    // yarp::sig::Vector               init_x; // initial cartes configuration
    // yarp::sig::Vector               init_o; // initial cartes orientation
    // yarp::sig::Matrix               traj_x; // endeff cartes pos
    // yarp::sig::Matrix               traj_xd;// endeff cartes vel
    //left arm
    yarp::sig::Matrix               traj_v_L; // joint velocity
    yarp::sig::Matrix               traj_s_L; // joint stiffness
    yarp::sig::Matrix               traj_d_L; // joint damping
    yarp::sig::Vector               init_q_L; // initial joint configuration
    yarp::sig::Vector               init_x_L; // initial cartes configuration
    yarp::sig::Vector               init_o_L; // initial cartes orientation
    yarp::sig::Matrix               traj_x_L; // endeff cartes pos
    yarp::sig::Matrix               traj_xd_L;// endeff cartes vel
    //right arm
    yarp::sig::Matrix               traj_v_R; // joint velocity
    yarp::sig::Matrix               traj_s_R; // joint stiffness
    yarp::sig::Matrix               traj_d_R; // joint damping
    yarp::sig::Vector               init_q_R; // initial joint configuration
    yarp::sig::Vector               init_x_R; // initial cartes configuration
    yarp::sig::Vector               init_o_R; // initial cartes orientation
    yarp::sig::Matrix               traj_x_R; // endeff cartes pos
    yarp::sig::Matrix               traj_xd_R;// endeff cartes vel
    
     //torso
    yarp::sig::Matrix               traj_v_T; // joint velocity
    yarp::sig::Matrix               traj_s_T; // joint stiffness
    yarp::sig::Matrix               traj_d_T; // joint damping
    yarp::sig::Vector               init_q_T; // initial joint configuration

    // filters
    iCub::ctrl::AWLinEstimator      *filter_xd_L,*filter_od_L,*filter_qd_L,
                                    *filter_xd_R,*filter_od_R,*filter_qd_R,
                                    *filter_qd_T;

    iCub::ctrl::AWQuadEstimator     *filter_xdd_L,*filter_odd_L,*filter_qdd_L,
                                    *filter_xdd_R,*filter_odd_R,*filter_qdd_R;     

    // for recording
    yarp::sig::Vector               cur_x_L,cur_o_L,cur_q_L,tmp_L;
    yarp::sig::Vector               cur_xd_L,cur_od_L,cur_qd_L;
    yarp::sig::Vector               cur_xdd_L,cur_odd_L,cur_qdd_L;
    yarp::sig::Vector               cur_x_R,cur_o_R,cur_q_R,tmp_R;
    yarp::sig::Vector               cur_xd_R,cur_od_R,cur_qd_R;
    yarp::sig::Vector               cur_xdd_R,cur_odd_R,cur_qdd_R;
    yarp::sig::Vector               cur_x_T,cur_o_T,cur_q_T,tmp_T;
    yarp::sig::Vector               cur_xd_T,cur_od_T,cur_qd_T;



    yarp::sig::Vector tmp;
    std::deque<yarp::sig::Vector>   recordedTraj;
    std::string fileName;

    int controlMode;
    int initMode;
    bool status;
    bool recordOnFile;
    std::string part;
    std::stringstream descriptor; 

    double oldTime;
    double curTime;
    int counter;
    int nbIter;
    int delay;

    void stopVelocity();
    bool stopJointsWithPosMove;

    void currentStatus(bool record=false);
    void filterEstimation();
    bool flushRecording();
    
    int nJointsArm;
    int nCartArm;
    int nJointsTorso;
    int nCartOrArm;
    int impedanceValue;

    trajectory_matrix *traj;
    


public:

    TrajPlayer(int rate,
               yarp::dev::PolyDriver *d_L, yarp::dev::PolyDriver *dc_L,
               yarp::dev::PolyDriver *d_R, yarp::dev::PolyDriver *dc_R,
               yarp::dev::PolyDriver *d_T,
               trajectory_matrix *, int contrlMode=0);

    bool threadInit();
    void run();
    void threadRelease();
    bool Play();
    bool goToInitPose();
    bool isOver();
    bool isInitPoseCartesian();
    bool isInitPoseJoint();
    void setRecording(std::string outFile);
    void setDelay(int del);
    void setInitMode (int mode);
    void setControlMode (int mode);
    void stop ();
    void displayTrajectory ();
    void clearTrajectory ();

public:
    //Display on iCubGui
    static Port guiPort;
};


#endif
