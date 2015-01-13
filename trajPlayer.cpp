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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include "trajPlayer.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

// utils for printing parameters
#define DSCP(V,oss) V; oss<< #V <<endl;
#define DSCPI(V,oss,i) V; oss<< #V <<"  i="<<i<<endl;


string i2s(int n)
{
    char buff [10];
    sprintf(buff,"%d",n);
    return string(buff);
}

//-----------------------------------------------------------

TrajPlayer::TrajPlayer(int rate, PolyDriver *d_L, PolyDriver *dc_L,
                       PolyDriver *d_R, PolyDriver *dc_R,
                       PolyDriver *d_T, trajectory_matrix *trajectories, int contrlMode)
    :RateThread(rate),dd_L(d_L),ddCart_L(dc_L),dd_R(d_R),ddCart_R(dc_R),
    dd_T(d_T),controlMode(contrlMode),status(STATUS_BLOCKED),counter(0)
{


    traj = trajectories;
    nJointsArm = trajectories->q_RA.cols ();
    nCartArm = trajectories->x_RA.cols ();
    nCartOrArm = trajectories->o_RA.cols ();
    nJointsTorso = trajectories->q_TO.cols ();

    nbIter = trajectories->x_RA.rows ();
    cout << endl;
    cout << "***INITIALIZE PLAYER***";
    cout << "- nJointsArm = " << nJointsArm << endl;
    cout << "- nCartArm = " << nCartArm << endl;
    cout << "- nCartOrArm = " << nCartOrArm << endl;
    cout << "- nJointsTorso = " << nJointsTorso << endl;

    // /// Initializing MATRICES
    // left arm
    traj_v_L.resize(nbIter, nJointsArm);                                traj_v_L.zero();
    traj_s_L = traj_d_L = traj_v_L;

    traj_x_L.resize(nbIter, nCartArm);                                  traj_x_L.zero();
    traj_xd_L = traj_x_L;
    
    init_q_L.resize(nJointsArm);                                        init_q_L.zero();
    init_x_L.resize(nCartArm);                                          init_x_L.zero();
    init_o_L.resize(nCartOrArm);                                        init_o_L.zero();
    
    // right arm
    traj_v_R.resize(nbIter, nJointsArm);                                traj_v_R.zero();
    traj_s_R = traj_d_R = traj_v_R;

    traj_x_R.resize(nbIter, nCartArm);                                  traj_x_R.zero();
    traj_xd_R = traj_x_R;

    init_q_R.resize(nJointsArm);                                        init_q_R.zero();
    init_x_R.resize(nCartArm);                                          init_x_R.zero();
    init_o_R.resize(nCartOrArm);                                        init_o_R.zero();

    // torso
    traj_v_T.resize(nbIter,nJointsTorso);                               traj_v_T.zero();
    traj_s_T = traj_d_T = traj_v_T;
    init_q_T.resize(nJointsTorso);                                      init_q_T.zero();

    // Initializing filters
    // left arm
    filter_xd_L = new AWLinEstimator(16,1.0);
    filter_od_L = new AWLinEstimator(16,1.0);
    filter_qd_L = new AWLinEstimator(16,1.0);
    filter_xdd_L = new AWQuadEstimator(25.0,1.0);
    filter_odd_L = new AWQuadEstimator(25.0,1.0);
    filter_qdd_L = new AWQuadEstimator(25.0,1.0);
    // right arm
    filter_xd_R = new AWLinEstimator(16,1.0);
    filter_od_R = new AWLinEstimator(16,1.0);
    filter_qd_R = new AWLinEstimator(16,1.0);
    filter_xdd_R = new AWQuadEstimator(25.0,1.0);
    filter_odd_R = new AWQuadEstimator(25.0,1.0);
    filter_qdd_R = new AWQuadEstimator(25.0,1.0);
    // Torso
    filter_qd_T = new AWLinEstimator(16,1.0);

    // useful stuff for recording
    cur_x_L.resize(3,0.0);        cur_xd_L = cur_xdd_L = cur_x_L;
    cur_o_L.resize(4,0.0);        cur_od_L = cur_odd_L = cur_o_L;
    cur_q_L.resize(nJointsArm,0.0);  cur_qd_L = cur_qdd_L = cur_q_L;
    cur_x_R.resize(3,0.0);        cur_xd_R = cur_xdd_R = cur_x_R;
    cur_o_R.resize(4,0.0);        cur_od_R = cur_odd_R = cur_o_R;
    cur_q_R.resize(nJointsArm,0.0);  cur_qd_R = cur_qdd_R = cur_q_R;

    stopJointsWithPosMove=false;

    recordedTraj.clear();
    tmp.clear(); tmp_L.clear(); tmp_R.clear();
    recordOnFile = true;
    descriptor.clear();
    delay = 0;


    for(int i=0; i<nJointsArm; i++)
    {
        traj_v_L.setCol(i,traj->qd_LA.getCol(i));
        init_q_L[i] = traj->q_LA[0][i];
        //
        traj_v_R.setCol(i,traj->qd_RA.getCol(i));
        init_q_R[i] = traj->q_RA[0][i];
        
    }
    
    for (int i=0; i < nJointsTorso; i++)
    {
        traj_v_T.setCol(i,traj->qd_TO.getCol(i));
        init_q_T[i] = traj->q_TO[0][i];
    }
    
     for(int i=0; i<3;i++)
     {
         init_x_L[i] = traj->x_LA[0][i];
         init_x_R[i] = traj->x_RA[0][i];
     }

     for(int i=0; i<4;i++)
     {
         init_o_L[i] = traj->o_LA[0][i];
         init_o_R[i] = traj->o_RA[0][i];
     }
     cout << "init q L   " << init_q_L.toString () << endl;

     cout << "init o L   "<< init_o_L.toString () << endl;

     cout << "init x L   "<< init_x_L.toString () << endl;

}

//-----------------------------------------------------------
bool TrajPlayer::threadInit()
{
    oldTime==Time::now();
    // LEFT
    if(!dd_L->view(icmd_L) || !dd_L->view(ipos_L) || !dd_L->view(iimp_L) ||
       !dd_L->view(ienc_L) || !dd_L->view(ivel_L))
    {
        cout<<"TrajPlayer: problem in acquiring interfaces of left_arm, aborting thread"<<endl;
        return false;
    }
    if(!ddCart_L->view(icrt_L))
    {
        cout<<"TrajPlayer: problem in acquiring Cartesian interface of left_arm, aborting thread"<<endl;
        return false;
    }
    //RIGHT
    if(!dd_R->view(icmd_R) || !dd_R->view(ipos_R) || !dd_R->view(iimp_R) ||
       !dd_R->view(ienc_R) || !dd_R->view(ivel_R))
    {
        cout<<"TrajPlayer: problem in acquiring interfaces of right_arm, aborting thread"<<endl;
        return false;
    }
    if(!ddCart_R->view(icrt_R))
    {
        cout<<"TrajPlayer: problem in acquiring Cartesian interface of right_arm, aborting thread"<<endl;
        return false;
    }
    //TORSO
    if(!dd_T->view(icmd_T) || !dd_T->view(ipos_T) || !dd_T->view(iimp_T) ||
       !dd_T->view(ienc_T) || !dd_T->view(ivel_T))
    {
        cout<<"TrajPlayer: problem in acquiring interfaces of Torso, aborting thread"<<endl;
        return false;
    }

 

    



    // else if(controlMode==CONTROLMODE_CART)
    // {
    //     // cartesian positions or positions+orientations
    //     cout << "Mode: cartesian" << endl;

    //     // file has x xd xdd y yd ydd z zd zdd
    //     for(int i=0; i<nJointsArm;i++)
    //     {
    //         // LEFT
    //         traj_x_L.setCol(i,traj->x_LA.getCol(i));
    //         //traj_xd_L.setCol(i,traj->getCol(3*i+1));
    //         init_x_L[i] = traj->x_LA[0][i];
    //         // RIGTH
    //         traj_x_R.setCol(i,traj->x_RA.getCol(i));
    //         //traj_xd_R.setCol(i, trajectories.jRA_matrix.getCol(3*ncartes+3*i+1));
    //         init_x_R[i] =  traj->x_RA[0][i];
    //     }
        
  

    // }
    // else
    // {
    
    //      cout<<" Default condition ---- if here it is bad"<<endl;



    // }

    /*
    // DEBUG: print trajectory
    cout<<"TrajPlayer acquired this velocity control trajectory:"<<endl;
    for(int i=0; i<traj_v.rows();i++)
        cout<<traj_v.getRow(i).toString()<<endl;
    */
    cout<<"TrajPlayer has this initial position: " << endl
        << "Rigth Arm" << endl
        << init_q_R.toString() << endl
        << "Left Arm" << endl
        << init_q_L.toString() << endl
        << "Torso" << endl
        << init_q_T.toString() << endl;


    return true;
}

//-----------------------------------------------------------
void TrajPlayer::threadRelease()
{
    cout<<"TrajPlayer: closing.."<<endl;
    // we don't close the polydrivers since it's the same for all

    if(filter_xd_L){ delete filter_xd_L;    filter_xd_L = 0;}
    if(filter_od_L){ delete filter_od_L;    filter_od_L = 0;}
    if(filter_qd_L){ delete filter_qd_L;    filter_qd_L = 0;}
    if(filter_xdd_L){ delete filter_xdd_L;  filter_xdd_L = 0;}
    if(filter_odd_L){ delete filter_odd_L;  filter_odd_L = 0;}
    if(filter_qdd_L){ delete filter_qdd_L;  filter_qdd_L = 0;}

    if(filter_xd_R){ delete filter_xd_R;    filter_xd_R = 0;}
    if(filter_od_R){ delete filter_od_R;    filter_od_R = 0;}
    if(filter_qd_R){ delete filter_qd_R;    filter_qd_R = 0;}
    if(filter_xdd_R){ delete filter_xdd_R;  filter_xdd_R = 0;}
    if(filter_odd_R){ delete filter_odd_R;  filter_odd_R = 0;}
    if(filter_qdd_R){ delete filter_qdd_R;  filter_qdd_R = 0;}

    if(filter_qd_T){ delete filter_qd_T;   filter_qd_T = 0;}


}

//-----------------------------------------------------------
void TrajPlayer::run()
{

    int i = 0;
    if(status==STATUS_PLAYING)
    {

        if (counter == 0)
        {
            curTime = 0;
            oldTime = Time::now ();
        }
        else
        {

            curTime = curTime + Time::now() - oldTime;
            oldTime = Time::now();
        }

        if (traj->timestamps[0][counter] <= curTime)
        {
            cout << counter <<"    -running ..." << endl;
        }

        //cout<<".. ++ " <<counter<<endl;
        //cout << traj->timestamps[0][counter] << " .... " << curTime << endl;

        //DEBUG
        //cout<<"counter = "<<counter<<endl;

        if(counter==0)
        {

  
            // if(controlMode==CONTROLMODE_VEL)
            // {
            //     cout<<"Starting velocity control"<<endl;
            //     for(i=0;i<nJointsArm;i++)
            //         icmd_L->setVelocityMode(i);
            //     for(i=0;i<nJointsArm;i++)
            //         icmd_R->setVelocityMode(i);
            //     for(i=0;i<nJointsTorso;i++)
            //         icmd_T->setVelocityMode(i);
            // }
            // else if((controlMode==CONTROLMODE_VELIMP_FIX) || (controlMode==CONTROLMODE_VELIMP_VAR))
            // {

            //     njointsImpedance = 5;
            //     cout<<"Starting impedance velocity control"<<endl;
            //     for(i=0;i<njointsImpedance;i++)
            //         iimp_R->setImpedance(i,traj_s_R[0][i],traj_d_R[0][i]);
            //     Time::delay(0.05);
            //     for(i=0;i<njointsImpedance;i++)
            //         iimp_L->setImpedance(i,traj_s_L[0][i],traj_d_L[0][i]);

            //     Time::delay(0.05);
            //     // not all joints support impedance velocity
            //     for(i=0;i<njointsImpedance;i++)
            //         icmd_L->setImpedanceVelocityMode(i);
            //     for(i=0;i<njointsImpedance;i++)
            //         icmd_R->setImpedanceVelocityMode(i);

            //     // the remaining joints are commanded in velocity
            //     for(i=njointsImpedance;i<nJointsArm;i++)
            //         icmd_R->setVelocityMode(i);
            //     for(i=njointsImpedance;i<nJointsArm;i++)
            //         icmd_L->setVelocityMode(i);
            //     for(i=0;i<nJointsTorso;i++)
            //         icmd_T->setVelocityMode(i);
            // }
            // counter = 1;


///////////////////////////////
               // initializing the control trajectories depending on control type

            if(controlMode==CONTROLMODE_VEL)
            {
            
                 cout<<"Mode: velocity"<<endl;
         

                for(int i=0; i<nJointsArm;i++)
                {
                    traj_v_L.setCol(i,traj->qd_LA.getCol(i));
                    init_q_L[i] = traj->q_LA[0][i];
                    init_x_L[i] = traj->x_LA[0][i];
                    //
                    traj_v_R.setCol(i,traj->qd_RA.getCol(i));
                    init_q_R[i] = traj->q_RA[0][i];
                    init_x_R[i] = traj->x_RA[0][i];
                }
                for (int i=0; i < nJointsTorso; i++)
                {
                    traj_v_T.setCol(i,traj->qd_TO.getCol(i));
                    init_q_T[i] = traj->q_TO[0][i];
                }

                for(int i=0; i<nCartOrArm;i++)
                {
                    init_o_L[i] = traj->o_LA[0][i];
                    init_o_R[i] = traj->o_RA[0][i];
                }
                traj_s_L.resize(1,nJointsArm); traj_s_L.zero();
                traj_d_L.resize(1,nJointsArm); traj_d_L.zero();

                njointsImpedance = 0;

            }
            else if(controlMode==CONTROLMODE_VELIMP_FIX)
            {

                cout << "Mode Velocity Impedance" << endl;
                for(int i=0; i<nJointsArm;i++)
                {
                    traj_v_L.setCol(i,traj->qd_LA.getCol(i));
                    init_q_L[i] = traj->q_LA[0][i];
                    init_x_L[i] = traj->x_LA[0][i];
                    //
                    traj_v_R.setCol(i,traj->qd_RA.getCol(i));
                    init_q_R[i] = traj->q_RA[0][i];
                    init_x_R[i] = traj->x_RA[0][i];
                }
                for (int i=0; i < nJointsTorso; i++)
                {
                    traj_v_T.setCol(i,traj->qd_TO.getCol(i));
                    init_q_T[i] = traj->q_TO[0][i];
                }

                for(int i=0; i<nCartOrArm;i++)
                {
                    init_o_L[i] = traj->o_LA[0][i];
                    init_o_R[i] = traj->o_RA[0][i];
                }
                traj_s_L.resize(1,nJointsArm); traj_s_L.zero();
                traj_d_L.resize(1,nJointsArm); traj_d_L.zero();

                if (impedanceValue == IMPSOFT)
                {
                    // only arm supported now
                    traj_s_L[0][0]=0.4;   traj_d_L[0][0]=0.03;
                    traj_s_L[0][1]=0.4;   traj_d_L[0][1]=0.03;
                    traj_s_L[0][2]=0.4;   traj_d_L[0][2]=0.03;
                    traj_s_L[0][3]=0.2;   traj_d_L[0][3]=0.01;
                    traj_s_L[0][4]=0.2;   traj_d_L[0][4]=0.00;
                    traj_s_R = traj_s_L;
                    traj_d_R = traj_d_L;
                }

                else if (impedanceValue == IMPMEDIUM)
                {
                    // only arm supported now
                    traj_s_L[0][0]=0.5;   traj_d_L[0][0]=0.04;
                    traj_s_L[0][1]=0.5;   traj_d_L[0][1]=0.04;
                    traj_s_L[0][2]=0.5;   traj_d_L[0][2]=0.04;
                    traj_s_L[0][3]=0.3;   traj_d_L[0][3]=0.02;
                    traj_s_L[0][4]=0.3;   traj_d_L[0][4]=0.00;
                    traj_s_R = traj_s_L;
                    traj_d_R = traj_d_L;
                }
                else
                {
                    // only arm supported now
                    traj_s_L[0][0]=0.6;   traj_d_L[0][0]=0.05;
                    traj_s_L[0][1]=0.6;   traj_d_L[0][1]=0.05;
                    traj_s_L[0][2]=0.6;   traj_d_L[0][2]=0.05;
                    traj_s_L[0][3]=0.4;   traj_d_L[0][3]=0.03;
                    traj_s_L[0][4]=0.4;   traj_d_L[0][4]=0.00;
                    traj_s_R = traj_s_L;
                    traj_d_R = traj_d_L;
                }
                njointsImpedance = 5;

                /////////////////////////////////////////////////////////////////

            }

            counter = 1;
        }
        else
        {
            if (counter < nbIter)
            { 
            // if there's variable impedance only
                if(controlMode==CONTROLMODE_VELIMP_VAR)
                {
                    for(int i=0; i<njointsImpedance;i++)
                        iimp_L->setImpedance(i,traj_s_L[counter][i],traj_d_L[counter][i]);
                    for(int i=0; i<njointsImpedance;i++)
                        iimp_R->setImpedance(i,traj_s_R[counter][i],traj_d_R[counter][i]);
                }


                // in all cases, send velocity commands 
                for(int i=0; i<nJointsArm;i++)
                {
                    // send velocity for RA
                    ivel_R->velocityMove(i,traj_v_R[counter][i]);
                   // cout << traj_v_R[i][counter] << " ";

                    // send velocity for LA
                    ivel_L->velocityMove(i,traj_v_L[counter][i]);
                   // cout << traj_v_L[counter][counter] << " ";
                }
                // send velocity for TO
                for(int i=0; i<nJointsTorso;i++)
                {
                   // cout << traj_v_T[counter][counter] << " ";
                    ivel_T->velocityMove(i,traj_v_T[counter][i]);
                }

                counter++;
            }


        }





    }
    //if we reached the end of the trajectory
    if(counter >= nbIter)
    {
        cout << "Mouvement Finished" << endl;
        counter=0;
        status = STATUS_BLOCKED;
        stopVelocity();
        oldTime=Time::now();
    }

}

void TrajPlayer::stop ()
{
    // cout<<"Stopping cartesian movements"<<endl;
    // if (icrt_R)
    // {
    //     icrt_R->getPose (cur_x_R, cur_o_R);
    //     icrt_R->goToPose (cur_x_R, cur_o_R);
    // }
    // if (icrt_L)
    // {
    //     icrt_L->getPose (cur_x_L, cur_o_L);
    //     icrt_L->goToPose (cur_x_L, cur_o_L);
    // }

    cout<<"Stopping cartesian controllers"<<endl;
    icrt_L->stopControl();
    icrt_R->stopControl();


    stopVelocity ();

    status = STATUS_BLOCKED;
    cout << "Mouvement Stopped" << endl;


}

//-----------------------------------------------------------
bool TrajPlayer::Play()
{
    if(status==STATUS_PLAYING)
        return false;
    else if(status==STATUS_BLOCKED)
    {
        counter=0;
        cout<<"TrajPlayer: Play!"<<endl;
        status = STATUS_PLAYING;
        return true;
    }
    else
        return false;
}

//-----------------------------------------------------------
void TrajPlayer::stopVelocity()
{

    if (ivel_R)
        for(int i=0;i<nJointsArm;i++)
            ivel_R->velocityMove(i,0.0);

    if (ivel_L)
        for(int i=0;i<nJointsArm;i++)
            ivel_L->velocityMove(i,0.0);

    if (ivel_T)
        for(int i=0;i<nJointsTorso;i++)
            ivel_T->velocityMove(i,0.0);

    Time::delay(0.3);

    if(stopJointsWithPosMove)
    {
        cout<<"Stopping joints"<<endl;

        Vector cur_pose_R(init_q_R.size());
        cur_pose_R.zero();
        for(int i=0; i<init_q_R.size();i++)
            ienc_R->getEncoder(i,&cur_pose_R[i]);

        Vector cur_pose_L(init_q_L.size());
        cur_pose_L.zero();
        for(int i=0; i<init_q_L.size();i++)
            ienc_L->getEncoder(i,&cur_pose_L[i]);

        Vector cur_pose_T(init_q_T.size());
        cur_pose_T.zero();
        for(int i=0; i<init_q_T.size();i++)
            ienc_T->getEncoder(i,&cur_pose_T[i]);

        for(int i=0; i<init_q_R.size();i++)
        {
            ipos_R->setRefSpeed(i,0);
            ipos_R->positionMove(i,init_q_R[i]);
        }
        for(int i=0; i<init_q_L.size();i++)
        {
            ipos_L->setRefSpeed(i,0);
            ipos_L->positionMove(i,init_q_L[i]);
        }
        for(int i=0; i<init_q_T.size();i++)
        {
            ipos_T->setRefSpeed(i,0);
            ipos_T->positionMove(i,init_q_T[i]);
        }
    }

}

//-----------------------------------------------------------
bool TrajPlayer::goToInitPose()
{
    if(status==STATUS_PLAYING)
        return false;
    else if(status==STATUS_BLOCKED)
    {
        if (initMode == INIT_MODE_RELATIVE)
        {
            cout<<"(RELATIVE)     Not going to init pose "<<endl;
            return true;
        }


        if ((initMode == INIT_MODE_CARTESIAN)||(initMode==INIT_MODE_CARTESIAN_JOINTS))
        {
            cout<<"TrajPlayer: going to init pose!"<<endl;

            // this must be done before sending the joints positions commands
            // for safety reason
            // otherwise we could make a movement too big and fast
            icrt_R->goToPose (init_x_R, init_o_R);
            cout<<"GoToPose "<<init_x_R.subVector(0,2).toString()<<endl;
            icrt_L->goToPose (init_x_L, init_o_L);
            cout<<"GoToPose "<<init_x_L.subVector(0,2).toString()<<endl;



            int countWait=0;
            while(isInitPoseCartesian()==false)
            {
                Time::delay(1.0);
                countWait++;
                if(countWait>=WAIT_N_INITPOSE)
                {
                    cout<<"ERROR: Player has been waiting too much ("<<countWait<<") for going to initial pose! Check thresholds!"<<endl;
                    // TEMPORARY
                    //status = STATUS_BLOCKED;
                    //return false;
                    if(initMode == INIT_MODE_CARTESIAN)
                        return true;
                  
                }
                if (STATUS_BLOCKED == status)
                {
                    cout << "Player is not running" << endl;
                    return true;
                }
            }
        }

        if (initMode == INIT_MODE_CARTESIAN_JOINTS)
        {
            Time::delay(1.5);

            cout<<"Moving joints"<<endl;

            for(int i=0; i<init_q_R.size();i++)
            {
                ipos_R->setRefSpeed(i,REF_SPEED);
                ipos_R->positionMove(i,init_q_R[i]);
            }
            for(int i=0; i<init_q_L.size();i++)
            {
                ipos_L->setRefSpeed(i,REF_SPEED);
                ipos_L->positionMove(i,init_q_L[i]);
            }
            for(int i=0; i<init_q_T.size();i++)
            {
                ipos_T->setRefSpeed(i,REF_SPEED);
                ipos_T->positionMove(i,init_q_T[i]);
            }
            int countWait=0;
            while(isInitPoseJoint()==false)
            {
                Time::delay(1.0);
                countWait++;
                if(countWait>=WAIT_N_INITPOSE)
                {
                    cout<<"ERROR: Player has been waiting too much ("<<countWait<<") for going to initial pose! Check thresholds!"<<endl;
                    // TEMPORARY
                    //status = STATUS_BLOCKED;
                    //return false;
                    return true;

                }
                if (STATUS_BLOCKED == status)
                {
                    cout << "Player is not running" << endl;
                    return true;
                }
            }
        }
        return true;
    }
}

//-----------------------------------------------------------
bool TrajPlayer::isOver()
{
    if(status==STATUS_PLAYING)
        return false;
    else if(status==STATUS_BLOCKED)
        return true;
    else
        return true;
}

//-----------------------------------------------------------
bool TrajPlayer::isInitPoseJoint()
{
    double dist1;
    double dist2;
    double dist3;

    // JOINT
    Vector cur_pose_R(init_q_R.size());
    cur_pose_R.zero();
    for(int i=0; i<init_q_R.size();i++)
        ienc_R->getEncoder(i,&cur_pose_R[i]);

    Vector cur_pose_L(init_q_L.size());
    cur_pose_L.zero();
    for(int i=0; i<init_q_L.size();i++)
        ienc_L->getEncoder(i,&cur_pose_L[i]);

    Vector cur_pose_T(init_q_T.size());
    cur_pose_T.zero();
    for(int i=0; i<init_q_T.size();i++)
        ienc_T->getEncoder(i,&cur_pose_T[i]);

    
    dist1=yarp::math::norm2(cur_pose_R - init_q_R);
    dist2=yarp::math::norm2(cur_pose_L - init_q_L);
    dist3=yarp::math::norm2(cur_pose_T - init_q_T);

    cout << "Initial position distance:" << endl;
    cout << "     Rigth Arm= " << dist1 << " ** Left Arm= " << dist2 << endl;
    //cout << "     Orientation: " << dist4 << " " << dist5 << endl;
    if(dist1<POSITION_THRESHOLD && dist2<POSITION_THRESHOLD)
    { 
        return true;
    }
    else
        return false;

}

bool TrajPlayer::isInitPoseCartesian()
{
    double dist1;
    double dist2;
    double dist3;

// CARTESIAN
    yarp::sig::Vector cur_x_R(init_x_R.size());
    yarp::sig::Vector cur_o_R(init_o_R.size());
    cur_x_R.zero();
    cur_o_R.zero();
    icrt_R->getPose (cur_x_R, cur_o_R);

    yarp::sig::Vector cur_x_L(init_x_L.size());
    yarp::sig::Vector cur_o_L(init_o_L.size());
    cur_x_L.zero();
    cur_o_L.zero();
     icrt_L->getPose (cur_x_L, cur_o_L);

    Vector cur_pose_T(init_q_T.size());
    cur_pose_T.zero();
    for(int i=0; i<init_q_T.size();i++)
        ienc_T->getEncoder(i,&cur_pose_T[i]);

    
    dist1 = yarp::math::norm2(cur_x_R - init_x_R);
    dist2 = yarp::math::norm2(cur_x_L - init_x_L);
    
    cout << "Initial position distance:" << endl;
    cout << "     Rigth Arm= " << dist1 << " ** Left Arm= " << dist2 << endl;
    if(dist1<POSITION_THRESHOLD && dist2<POSITION_THRESHOLD)
    { 
        return true;
    }
    else
        return false;
    
}

//-----------------------------------------------------------
void TrajPlayer::currentStatus(bool record)
{
/*    icrt->getPose(cur_x,cur_o);
    ienc->getEncoders(cur_q.data());
    // estimation of xd, od, qd, xdd, odd, qdd
    filterEstimation();

    if(record==true)
    {
        int c=0;
        // saving data
        tmp.clear();
        descriptor.clear();
        descriptor.str("");

        // (q qd qdd) (x xd xdd)
        tmp.resize(9+3*njoints);
        for(int i=0; i<njoints;i++)
        {
            tmp[c++] = DSCPI(cur_q[i],descriptor,i);
            tmp[c++] = DSCPI(cur_qd[i],descriptor,i);
            tmp[c++] = DSCPI(cur_qdd[i],descriptor,i);
        }
        tmp[c++] = DSCP(cur_x[0],descriptor);
        tmp[c++] = DSCP(cur_xd[0],descriptor);
        tmp[c++] = DSCP(cur_xdd[0],descriptor);
        tmp[c++] = DSCP(cur_x[1],descriptor);
        tmp[c++] = DSCP(cur_xd[1],descriptor);
        tmp[c++] = DSCP(cur_xdd[1],descriptor);
        tmp[c++] = DSCP(cur_x[2],descriptor);
        tmp[c++] = DSCP(cur_xd[2],descriptor);
        tmp[c++] = DSCP(cur_xdd[2],descriptor);

        recordedTraj.push_back(tmp);
    }
    */
}

//-----------------------------------------------------------
void TrajPlayer::filterEstimation()
{
    AWPolyElement el;
    el.time=Time::now();

    el.data = cur_x_L;
    cur_xd_L = filter_xd_L->estimate(el);
    cur_xdd_L= filter_xdd_L->estimate(el);

    el.data = cur_o_L;
    cur_od_L = filter_od_L->estimate(el);
    cur_odd_L= filter_odd_L->estimate(el);

    el.data = cur_q_L;
    cur_qd_L = filter_qd_L->estimate(el);
    cur_qdd_L= filter_qdd_L->estimate(el);

    el.data = cur_x_R;
    cur_xd_R = filter_xd_R->estimate(el);
    cur_xdd_R= filter_xdd_R->estimate(el);

    el.data = cur_o_R;
    cur_od_R = filter_od_R->estimate(el);
    cur_odd_R= filter_odd_R->estimate(el);

    el.data = cur_q_R;
    cur_qd_R = filter_qd_R->estimate(el);
    cur_qdd_R= filter_qdd_R->estimate(el);

}

//-----------------------------------------------------------
bool TrajPlayer::flushRecording()
{
    if(recordOnFile==true)
    {
        int totalRec = recordedTraj.size();

        ofstream out;
        out.open("taskPlayerResult.record");
        if(!out)
        {
            cout<<"There were errors saving the trajectory file: "<<fileName<<endl
                <<"Aborting."<<endl;
            return false;
        }
        else
            cout<<"Saving "<<totalRec<<" items on "<<fileName<<endl;

        tmp.clear();

        for(int i=0; i<totalRec;i++)
        {
            tmp = recordedTraj.front();
            recordedTraj.pop_front();
            out << tmp.toString() << endl;
        }

        recordedTraj.clear(); tmp.clear();
        out.close();

        out.open(string(fileName+".info").c_str());
        if(!out)
        {
            cout<<"There were errors saving the information file: "<<string(fileName+".info")<<endl
                <<"Aborting."<<endl;
            return false;
        }
        else
            cout<<"Saving info file on "<<string(fileName+".info")<<endl;

        out<<descriptor.str()<<endl;
        out.close();

        return true;
    }

    cout<<"Recording is not currently enabled for this player! "<<endl;
    return false;

}

void TrajPlayer::clearTrajectory ()
{
     yarp::os::Bottle obj;

     obj.addString("reset");
     guiPort.write (obj);
}

void TrajPlayer::displayTrajectory ()
{
    cout << "Display Trajectory" << endl;

    for (int i = 0; i < nbIter; i += 10)
    {
        double progress = (double) i/ (double) nbIter * 255;
        cout << progress << " " << (int)(progress) % 255 << endl;
        yarp::os::Bottle obj;
        string objname = "posLA"+i2s(i);
         obj.addString("object"); // command to add/update an object
         obj.addString(objname.c_str ());

         // object dimensions in millimiters 
         obj.addDouble(5);
         obj.addDouble(5);
         obj.addDouble(5);
         
         // object position in millimiters
         // reference frame: X=fwd, Y=left, Z=up
         obj.addDouble(traj->x_LA[i][0] * 1000);
         obj.addDouble(traj->x_LA[i][1] * 1000);
         obj.addDouble(traj->x_LA[i][2] * 1000);

         // object orientation (roll, pitch, yaw) in degrees
         obj.addDouble(0);
         obj.addDouble(0);
         obj.addDouble(0);
         
         // object color (0-255)
         obj.addInt( (int)(progress) % 255);
         obj.addInt(0);
         obj.addInt(255 - (int)(progress) % 255);
         // transparency (0.0=invisible 1.0=solid)
         obj.addDouble(1);

         //cout << "Object at: " << traj->x_LA[i][0] << " " << traj->x_LA[i][1] << " " << traj->x_LA[i][2] << endl;
         cout << obj.toString () << endl;
        guiPort.write (obj);
    }
    for (int i = 0; i < nbIter; i += 10)
    {
        double progress = (double) i/ (double) nbIter * 255;
        cout << progress << " " << (int)(progress) % 255 << endl;
        yarp::os::Bottle obj;
        string objname = "posRA"+i2s(i);
         obj.addString("object"); // command to add/update an object
         obj.addString(objname.c_str ());

         // object dimensions in millimiters 
         obj.addDouble(5);
         obj.addDouble(5);
         obj.addDouble(5);
         
         // object position in millimiters
         // reference frame: X=fwd, Y=left, Z=up
         obj.addDouble(traj->x_RA[i][0] * 1000);
         obj.addDouble(traj->x_RA[i][1] * 1000);
         obj.addDouble(traj->x_RA[i][2] * 1000);

         // object orientation (roll, pitch, yaw) in degrees
         obj.addDouble(0);
         obj.addDouble(0);
         obj.addDouble(0);
         
         // object color (0-255)
         obj.addInt( (int)(progress) % 255);
         obj.addInt(0);
         obj.addInt(255 - (int)(progress) % 255);
         // transparency (0.0=invisible 1.0=solid)
         obj.addDouble(1);

         //cout << "Object at: " << traj->x_LA[i][0] << " " << traj->x_LA[i][1] << " " << traj->x_LA[i][2] << endl;
         cout << obj.toString () << endl;
         guiPort.write (obj);
    }
}

//----------------------------------------------------------
void TrajPlayer::setRecording(string outFile)
{
    recordOnFile = true;
    fileName = outFile;
    cout<<"TrajPlayer will save on file "<<fileName<<endl;
}

//----------------------------------------------------------
void TrajPlayer::setDelay(int del)
{
    delay = del;
}

void TrajPlayer::setInitMode(int mode)
{
    initMode = mode;
}
void TrajPlayer::setControlMode(int mode)
{
    switch (mode)
    {
        case 0:
            controlMode = CONTROLMODE_VEL;
            impedanceValue = IMPSOFT;
            break;
        case 1:
            controlMode = CONTROLMODE_VELIMP_FIX;
            impedanceValue = IMPSOFT;
            break;
        case 2:
            controlMode = CONTROLMODE_VELIMP_FIX;
            impedanceValue = IMPMEDIUM;
            break;
        case 3:
            controlMode = CONTROLMODE_VELIMP_FIX;
            impedanceValue = IMPHARD;
            break;
        default:
            controlMode = CONTROLMODE_VEL;
            impedanceValue = IMPSOFT;
            break;
    }
    
}

