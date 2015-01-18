/*
 * Copyright (C) 2015 CODYCO Project
 * Author: Serena Ivaldi <serena.ivaldi@inria.fr>
 * website: www.codyco.eu
 *
 * Copyright (C) 2013-2014 MACSi & EDHHI Projects
 * Author: Serena Ivaldi, Charles Ballarini
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
 

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

//#include <macsi/modHelp/modHelp.h>
//#include <macsi/objects/objects.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Math.h>
#include <gsl/gsl_math.h>
#include <deque>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <map>
#include <vector>

#include "trajPlayer.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
//using namespace macsi::modHelp;
//using namespace macsi::objects;
using namespace std;

// necessary for cartesian interface
YARP_DECLARE_DEVICES(icubmod)

// utils for printing parameters
#define DSCPA(V) cout<<"  "<< #V <<" : "<<V<<endl;
#define DSCPAv(V) cout<<"  "<< #V <<" : "<<V.toString()<<endl;


//==> from modHelp

#define displayValue(V) cout<<" "<< #V <<" : "<<V<<endl;
#define displayNameValue(S,V) cout<<" "<< S <<" : "<<V<<endl;
#define displayVector(V) cout<<" "<< #V <<" : "<<V.toString()<<endl;
#define displayNameVector(S,V) cout<<" "<< S <<" : "<<V.toString()<<endl;

 void readString(ResourceFinder &rf, string name, string &v, string vdefault)
{
if(rf.check(name.c_str()))
{
v = rf.find(name.c_str()).asString();
}
else
{
v = vdefault;
cout<<"Could not find value for "<<name<<". "
<<"Setting default "<<vdefault<<endl;
}
displayNameValue(name,v);
}


void readInt(ResourceFinder &rf, string name, int &v, int vdefault)
{
if(rf.check(name.c_str()))
{
v = rf.find(name.c_str()).asInt();
}
else
{
v = vdefault;
cout<<"Could not find value for "<<name<<". "
<<"Setting default "<<vdefault<<endl;
}
displayNameValue(name,v);
}


//==>end



// global shared port for communicating with the iCub_GUI
Port TrajPlayer::guiPort = Port ();



// MODULE
//-------------------------------------
class TrajModule: public RFModule
{
private:

    //
    std::map<string, TrajPlayer*> players;
    bool isPlaying;
    //
    bool verbose;
    string name;
    string robot;
    string file;
    int rate;
    // Setting for joints
    int nJointsArm;// = 16;
    int nCartArm;// = 3;
    int nCartOrArm;// = 4;
    int nJointsTorso;// = 3;


    string currentFile;
    string controlModeS;
    int controlMode;
    int delay;
    Port rpc;
    //
    // left arm
    Property options_L, optionsCart_L;
    PolyDriver *dd_L;
    PolyDriver *ddCart_L;
    IPositionControl *ipos_L;
    IEncoders *ienc_L;
    IControlMode *imode_L;
    IImpedanceControl *iimp_L;
    ICartesianControl *icrt_L;
    // Vector orientHand_L;
    // right arm
    Property options_R, optionsCart_R;
    PolyDriver *dd_R;
    PolyDriver *ddCart_R;
    IPositionControl *ipos_R;
    IEncoders *ienc_R;
    IControlMode *imode_R;
    IImpedanceControl *iimp_R;
    ICartesianControl *icrt_R;
    // Vector orientHand_R;

    // torso
    Property options_T;
    PolyDriver *dd_T;
    IPositionControl *ipos_T;
    IEncoders *ienc_T;
    IControlMode *imode_T;
    IImpedanceControl *iimp_T;
    //
    Matrix **trajectory;


    //---------------------------------------------------------
    string i2s(int n)
    {
        char buff [10];
        sprintf(buff,"%d",n);
        return string(buff);
    }

    //---------------------------------------------------------
    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) 
    {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    //---------------------------------------------------------
    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        split(s, delim, elems);
        return elems;
    }

public:

    //---------------------------------------------------------
    TrajModule()
    {
        verbose = true;
        dd_L=dd_R=dd_T=0;
        ddCart_L=ddCart_R=0;
        trajectory=0;
    }

    //---------------------------------------------------------
    bool openDriversArm(Property &options, string part, PolyDriver *pd, IPositionControl *ipos, IEncoders *ienc, IControlMode *imode, IImpedanceControl *iimp)
    {
        // open the device drivers
        options.put("device","remote_controlboard");
        options.put("local",string("/"+name+"/"+part).c_str());
        options.put("remote",string("/"+robot+"/"+part).c_str());
        if(!pd->open(options))
        {
            cout<<"Problems connecting to the remote driver of "<<part<<endl;
            close();
            return false;
        }
        if(!pd->view(imode) || !pd->view(ienc) || !pd->view(ipos) || !pd->view(iimp) )
        {
            cout<<"Problems acquiring interfaces for "<<part<<endl;
            close();
            return false;
        }
        return true;

    }

    //---------------------------------------------------------    
    bool openDriversTorso(Property &options, PolyDriver *pd, IPositionControl *ipos, IEncoders *ienc, IControlMode *imode, IImpedanceControl *iimp)
    {
        // open the device drivers
        options.put("device","remote_controlboard");
        options.put("local",string("/"+name+"torso").c_str());
        options.put("remote",string("/"+robot+"/torso").c_str());
        if(!pd->open(options))
        {
            cout<<"Problems connecting to the remote driver of torso"<<endl;
            close();
            return false;
        }
        if(!pd->view(imode) || !pd->view(ienc) || !pd->view(ipos) || !pd->view(iimp) )
        {
            cout<<"Problems acquiring interfaces for torso"<<endl;
            close();
            return false;
        }
        return true;

    }


    //---------------------------------------------------------
    bool openCartesians(Property &optionsCart, string part, PolyDriver *ddCart, ICartesianControl *icrt)
    {
        // open the cartesian driver
        optionsCart.put("device","cartesiancontrollerclient");
        optionsCart.put("remote",string("/"+robot+"/cartesianController/"+part).c_str());
        optionsCart.put("local",string("/"+name+"/"+part+"/cartesian").c_str());

	cout<<"++++ cartesian driver: open " <<endl;
        if (!ddCart->open(optionsCart))
        {
            cout<<"Problems connecting to the Cartesian interface of "<<part<<endl;
            close();
            return false;
        }
	cout<<"++++ cartesian driver: is valid " <<endl;
        if(!ddCart->isValid())
        {
            cout<<"Invalid Cartesian interface for "<<part<<endl;
            close();
            return false;
        }
	cout<<"++++ cartesian driver: view " <<endl;
        if(!ddCart->view(icrt))
        {
            cout<<"Problems acquiring the Cartesian interface of "<<part<<endl;
            close();
            return false;
        }
	
	return true;
    }

    //---------------------------------------------------------
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();
        isPlaying = false;

        if(rf.check("name"))
            name    = rf.find("name").asString();
        else
            name    = "taskPlayer";
        //....................................................
        if(rf.check("robot"))
            robot   = rf.find("robot").asString();
        else
            robot   = "icubSim"; //by default simulator, so we dont break the real one
        //....................................................
        if(rf.check("file"))
            file    = rf.find("file").asString();
        else
            file    = "test";
        //....................................................
        if(rf.check("rate"))
            rate    = rf.find("rate").asInt();
        else
            rate    = 10; //default 10ms
        //....................................................
        if(rf.check("nJointsArm"))
            nJointsArm = rf.find("nJointsArm").asInt();
        else
            nJointsArm = 16; //default 16
        //....................................................
        if(rf.check("nCartArm"))
            nCartArm = rf.find("nCartArm").asInt();
        else
            nCartArm = 3; //default 3 = x y z - no orientation
        //....................................................
        if(rf.check("nCartOrArm"))
            nCartOrArm = rf.find("nCartOrArm").asInt();
        else
            nCartOrArm = 4; //default 3 = x y z - no orientation
        //....................................................
        if(rf.check("nJointsTorso"))
            nJointsTorso = rf.find("nJointsTorso").asInt();
        else
            nJointsTorso = 3; //default 3 = x y z - no orientation
        //....................................................
        if(rf.check("control"))
            controlModeS    = rf.find("control").asString();
        else
            controlModeS    = "velocity"; //by default only velocity
        if(controlModeS=="velocity") controlMode = CONTROLMODE_VEL;
        else if(controlModeS=="velocity_impedance_fixed") controlMode = CONTROLMODE_VELIMP_FIX;
        else if(controlModeS=="velocity_impedance_variable") controlMode = CONTROLMODE_VELIMP_VAR;
//        else if(controlModeS=="cartesian") controlMode = CONTROLMODE_CART;
        else
        {
            cout<<"Warning: control mode "<<controlModeS<<" not supported. Setting velocity by default"<<endl;
            controlMode = CONTROLMODE_VEL;
        } 
        //....................................................
        if(rf.check("delay"))
            delay   = rf.find("delay").asInt();
        else
            delay   = 0; // by default, no delay in recording
        //....................................................
        // orientHand_L.resize(4,0.0);
        // if(rf.check("orientHandLeft"))
        // {
        //     Bottle *tmpL = rf.find("orientHandLeft").asList();
        //     for(int i=0; i<tmpL->size();i++)
        //         orientHand_L[i] = tmpL->get(i).asDouble();
        // }
        // else
        // {
        //     cout<<"Warning: couldn't find orientation for left arm. Setting default"<<endl;
        //     orientHand_L[0] = -0.185191;
        //     orientHand_L[1] =  0.585074;
        //     orientHand_L[2] = -0.789552;
        //     orientHand_L[3] =  3.124785;

        // }

        // //....................................................
        // orientHand_R.resize(4,0.0);
        // if(rf.check("orientHandRight"))
        // {
        //     Bottle *tmpR = rf.find("orientHandRight").asList();
        //     for(int i=0; i<tmpR->size();i++)
        //         orientHand_R[i] = tmpR->get(i).asDouble();
        // }
        // else
        // {
        //     cout<<"Warning: couldn't find orientation for right arm. Setting default"<<endl;
        //     orientHand_R[0] = -0.000203;
        //     orientHand_R[1] = -0.808821;
        //     orientHand_R[2] =  0.588055;
        //     orientHand_R[3] =  2.760923;

        // }
        //....................................................

        cout<<"Parameters from init file: "<<endl;
        DSCPA(name);
        DSCPA(robot);
        DSCPA(file);
        DSCPA(rate);
        DSCPA(nJointsArm);
        DSCPA(nCartArm);
        DSCPA(nCartOrArm);
        DSCPA(nJointsTorso);
        DSCPA(controlModeS);
        DSCPA(delay);
        // DSCPAv(orientHand_L);
        // DSCPAv(orientHand_R);

        // open the device drivers
        
        bool isDDOpen = false;
        dd_L = new PolyDriver;
        dd_R = new PolyDriver;
        dd_T = new PolyDriver;
	
	// left arm
	cout<<"** Opening left arm drivers"<<endl;
        isDDOpen = openDriversArm(options_L, "left_arm", dd_L, ipos_L, ienc_L, imode_L, iimp_L);

        if(isDDOpen==false)
        {
            cout << "Problem in opening Left Arm driver" << endl;
            return false;
        }

        //right arm
	cout<<"** Opening right arm drivers"<<endl;
        isDDOpen = openDriversArm(options_R, "right_arm", dd_R, ipos_R, ienc_R, imode_R, iimp_R);
        if(isDDOpen==false)
        {
            cout << "Problem in opening Right Arm driver" << endl;
            return false;
        }
        //torso
	cout<<"** Opening torso drivers"<<endl;
        isDDOpen = openDriversTorso(options_T, dd_T, ipos_T, ienc_T, imode_T, iimp_T); 
        if(isDDOpen==false)
        {
            cout << "Problem in opening Torso driver" << endl;
            return false;
        }
        

        // open the cartesian drivers
 
        ddCart_L = new PolyDriver ();
        ddCart_R =  new PolyDriver ();
       // left arm
	cout<<"** Opening left arm cartesian"<<endl;
        if(openCartesians(optionsCart_L, "left_arm", ddCart_L, icrt_L)==false)
            return false;
        // right arm
	cout<<"** Opening right arm cartesian"<<endl;
        if(openCartesians(optionsCart_R, "right_arm", ddCart_R, icrt_R)==false)
            return false;


	cout<<"** All drivers open **"<<endl;

        //Start listening
        rpc.open(string("/"+name+"/rpc:i").c_str());
        attach(rpc);

	cout<<"** rpc port open **"<<endl;

        return true;

    }


    //---------------------------------------------------------
    // LOADING TRAJECTORIES
    //---------------------------------------------------------
    void LoadFile (string filepath)
    {
        cout<<"Reading trajectories from files "<<endl;
        // Load all information of trajectories file.
        trajectory_matrix *trajectories = new trajectory_matrix ();

        deque<double> tmpTrj; double tmp;
        ifstream inputFile;
        string filename;
        for(int i=0; i<1; i++)
        {
            filename.clear();

            //iFile=string(rf.getContextPath().c_str())+"/"+file+i2s(i+1)+"_dmp_output.txt";
            filename = string (filepath);

	    cout<<"++++ Loading trajectory file from path: "<<filepath<<endl;

            inputFile.open(filename.c_str());
            if (!inputFile.is_open ())
            {
                cout << "ERROR: Can't open file: " << filename << endl;
                return;
            }
            string l;
            
            // number of line in file 
            int nbIter = 0;

            while (getline (inputFile, l))
            {
                nbIter++;
            }
            cout << "INFO: "<< filename << " is a record of " << nbIter << " iterations" << endl;
            inputFile.clear();
            inputFile.seekg(0, ios::beg);


            //creation of trajectories
            // Creation of matrix
            trajectories->x_RA = Matrix (nbIter, nCartArm);
            trajectories->x_LA = Matrix (nbIter, nCartArm);

            trajectories->o_LA = Matrix (nbIter, nCartOrArm);
            trajectories->o_RA = Matrix (nbIter, nCartOrArm);

            trajectories->q_RA = Matrix (nbIter, nJointsArm);
            trajectories->q_LA = Matrix (nbIter, nJointsArm);
            trajectories->q_TO = Matrix (nbIter, nJointsTorso);

            // Creation of dmatrix
            trajectories->xd_RA = Matrix (nbIter, nCartArm);
            trajectories->xd_LA = Matrix (nbIter, nCartArm);

            trajectories->od_LA = Matrix (nbIter, nCartOrArm);
            trajectories->od_RA = Matrix (nbIter, nCartOrArm);

            trajectories->qd_RA = Matrix (nbIter, nJointsArm);
            trajectories->qd_LA = Matrix (nbIter, nJointsArm);
            trajectories->qd_TO = Matrix (nbIter, nJointsTorso);


            trajectories->timestamps = Matrix (nbIter, 1);


            //initialisation of trajectories
            trajectories->x_RA.zero ();
            trajectories->x_LA.zero ();

            trajectories->o_LA.zero ();
            trajectories->o_RA.zero ();

            trajectories->q_RA.zero ();
            trajectories->q_LA.zero ();
            trajectories->q_TO.zero ();

            // Initialize dmatrix
            trajectories->x_RA.zero ();
            trajectories->xd_LA.zero ();

            trajectories->od_LA.zero ();
            trajectories->od_RA.zero ();

            trajectories->qd_RA.zero ();
            trajectories->qd_LA.zero ();
            trajectories->qd_TO.zero ();


            trajectories->timestamps.zero ();
            int c = 0;
            while (getline (inputFile, l))
            {
                printf ("Load file %s : \r%d / %d", filename.c_str (), c + 1, nbIter);
                stringstream line;
                line << l;
                //timestamp
                line >> (trajectories->timestamps[c][0]);
                //RA
                for (int i = 0; i < nCartArm; i++)
                    line >> trajectories->x_RA[c][i];
                for (int i = 0; i < nCartOrArm; i++)
                    line >> trajectories->o_RA[c][i];
                for (int i = 0; i < nJointsArm; i++)
                    line >> trajectories->q_RA[c][i];
                for (int i = 0; i < nCartArm; i++)
                    line >> trajectories->xd_RA[c][i];
                for (int i = 0; i < nCartOrArm; i++)
                    line >> trajectories->od_RA[c][i];
                for (int i = 0; i < nJointsArm; i++)
                    line >> trajectories->qd_RA[c][i];

                //LA
                for (int i = 0; i < nCartArm; i++)
                    line >> trajectories->x_LA[c][i];
                for (int i = 0; i < nCartOrArm; i++)
                    line >> trajectories->o_LA[c][i];
                for (int i = 0; i < nJointsArm; i++)
                    line >> trajectories->q_LA[c][i];
                for (int i = 0; i < nCartArm; i++)
                    line >> trajectories->xd_LA[c][i];
                for (int i = 0; i < nCartOrArm; i++)
                    line >> trajectories->od_LA[c][i];
                for (int i = 0; i < nJointsArm; i++)
                    line >> trajectories->qd_LA[c][i];

                //TO

                for (int i = 0; i < nJointsTorso; i++)
                    line >> trajectories->q_TO[c][i];
                for (int i = 0; i < nJointsTorso; i++)
                    line >> trajectories->qd_TO[c][i];
                c++;

            }

        }
        

        // creating all the threads in charge of reproducing the trajectories
        // player = new TrajPlayer*[1];

        //Create and add player to players
        TrajPlayer *player = new TrajPlayer(rate, dd_L, ddCart_L, dd_R, ddCart_R, dd_T, trajectories, controlMode);
        players.insert (std::pair<string, TrajPlayer*> (filepath, player));
    }

    //---------------------------------------------------------
    // PLAYING TRAJECTORIES
    //---------------------------------------------------------
    void PlayFile (string filename, int initMode, int controlMode)
    {
        if (isPlaying)
        {
            cout << "Already Running:" << currentFile << endl;
            return;
        }
        if (players.find (filename) == players.end ())
            cout << "File not loaded: " << filename << endl;
        else
        {
            cout << "Options:" << endl;
            cout << "   initMode: " << initMode << endl;
            cout << "   controlMode: " << controlMode << endl;
            players.find(filename)->second->setInitMode (initMode);
            players.find(filename)->second->setControlMode (controlMode);
            if(!players.find(filename)->second->start())
            {
                cout<<"Could not start thread:  " << filename <<endl
                                <<"Aborting."<<endl;
            }
            else
            {

                currentFile = filename;
                isPlaying = true;
            }
        }
    }

    //---------------------------------------------------------
    // DISPLAY TRAJECTORIES
    //---------------------------------------------------------
    void displayTrajectory (string filename)
    {
        if (players.find (filename) == players.end ())
            cout << "File not loaded" << endl;
        else
            players.find(filename)->second->displayTrajectory();
    }

    //---------------------------------------------------------
    // CLEAR TRAJECTORIES
    //---------------------------------------------------------
    void clearTrajectory (string filename)
    {
        if (players.find (filename) == players.end ())
            cout << "File not loaded" << endl;
        else
            players.find(filename)->second->clearTrajectory();
    }

    //---------------------------------------------------------
    // STOP MOTION
    //---------------------------------------------------------
    void stop_all ()
    {
        if (players.find (currentFile) != players.end ())
        {
            cout << "Stop: " << currentFile << endl;
            players[currentFile]->stop ();
        }
        isPlaying = false;
    }


    //---------------------------------------------------------
    // RPC - COMMANDS
    //---------------------------------------------------------
    bool respond(const Bottle& command, Bottle& reply)
    {
        string c = command.toString().c_str ();
        if (c.at (0) == '"')
            c = c.substr (1, c.size () - 2);


        std::vector<std::string> tokens = split(c, ' ');
        if (tokens[0] == "quit")
            return false;
        if (tokens[0] == "Load")
        {   
            if (tokens.size () == 2)
            {
                LoadFile (tokens[1]);
            }
            else
                cout << "You need to specify a file to load" << endl;
            return true;
        }   
        if (tokens[0] == "Play")
        {   
            if (tokens.size () == 4)
            {
                PlayFile (tokens[1], atoi(tokens[2].c_str ()), atoi(tokens[3].c_str ()));
            }
               
            else
            {
                cout << "Problem in command: 4 arg needed, " << tokens.size () << " given." << endl;
                cout << "Help: Play filename initMode controlMod" << endl;
            }
            return true;
        }
        if (tokens[0] == "STOP")
        {
            stop_all ();
            return true;
        }
        if (tokens[0] == "Clear")
        {
            if (tokens.size () == 2)
                clearTrajectory (tokens[1]);
            else
                cout << "You have to specify which trajectory to clean" << endl;
            return true;
        }

        if (tokens[0] == "Display")
        {
            if (tokens.size () == 2)
                displayTrajectory (tokens[1]);
            else
                cout << "You have to specify which trajectory to display" << endl;
            return true;
        }

        cout << "Command '" << tokens[0] << "' not known" << endl; 

        return true;
        
    }


    //---------------------------------------------------------
    bool close()
    {
        if(verbose) 
            cout << "Closing threads" << endl;
        players.clear ();
       
        
        
        if(verbose) 
            cout << "Closing polydrivers" << endl;
        if(dd_R)      {delete dd_R; dd_R=0; }
        if(ddCart_R)  {delete ddCart_R; ddCart_R=0;}
        if(dd_L)      {delete dd_L; dd_L=0; }
        if(ddCart_L)  {delete ddCart_L; ddCart_L=0;}
        if(dd_T)      {delete dd_T; dd_T=0;}

        return true;
    }

    //---------------------------------------------------------
    double getPeriod()    { return 1.0;  }


        // //starting the threads: they'll start as blocked
        // for(int i=0; i<trials; i++)
        // {
        //     if(verbose) cout<<"Starting player # "<<i<<endl;
        //     if(!player[i]->start())
        //     {
        //         delete player[i];
        //         cout<<"Could not start thread # "<<i<<endl
        //             <<"Aborting."<<endl;
        //         return false;
        //     }

        //     Time::delay(0.5);
        // }
    //---------------------------------------------------------
    bool   updateModule()
    {
        if (isPlaying)
        {
            if( players[currentFile]->goToInitPose() == false)
            {
                cout<<"Couldn't init trajectory: " << currentFile << endl;
                return true;
            }
            else
            {
                // execute trajectory
                if (players[currentFile]->Play())
                {
                    // wait for trajectory to be done
                    while(players[currentFile]->isOver()==false)
                        Time::delay(0.1);
                    // trajectory has been executed
                }
                else
                {
                    cout << "Error: Problem playing " << currentFile << endl;
                    return false;
                }
            }
            isPlaying = false;
        }
        return true;
    }

    //---------------------------------------------------------
};




//---------------------------------------------------------
//                  MAIN
//---------------------------------------------------------
int main(int argc, char *argv[])
{
    //necessary for Cartesian interface
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }

    string moduleName;

    ResourceFinder rf;
    //retrieve information for the list of parts
    rf.setVerbose(true);
    rf.setDefaultContext("taskRecorder");
    rf.setDefaultConfigFile("taskPlayer.ini");
    rf.configure(argc,argv);

    moduleName = rf.find("moduleName").asString();
    if (moduleName == "")
        moduleName = "taskPlayer";
    

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--from   fileName: input configuration file" << endl;
        cout << "\t--context dirName: resource finder context"  << endl;

        return 0;
    }


    TrajPlayer::guiPort.open(string("/"+moduleName+"/display:o").c_str());
    cout<<"++++ Automatic connection to iCubGui (if exists)"<<endl;
    Network::connect(string("/"+moduleName+"/display:o").c_str(),"/iCubGui/objects");

    TrajModule mod;
    mod.runModule(rf);

    return 0;
}
