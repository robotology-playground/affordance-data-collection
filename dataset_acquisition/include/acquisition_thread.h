/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef __ACQUISITION_THREAD_H__
#define __ACQUISITION_THREAD_H__

#include <string>
#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/all.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>

#include <iostream>
#include <fstream>

#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>


#include <yarp/os/RateThread.h>
#include <yarp/os/RpcClient.h>

#include "helpers.h"

// change these values for the individual cases; e.g. for arm encoders, use only 17
#define NUM_INPUTS      46 // 41 encoders, 3 pos, 2 face
#define NUM_ENCODERS    41
#define NUM_3DPOS       3
#define NUM_FACE        2
#define NUM_OBJECTS     18
#define NUM_STEPS       100


using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace cv;


class dataset_entry 
{
public:
    string img_name;
    std::vector<double> Pos3D;
    std::vector<int> one_hot_labels;
    string sentence;
    std::vector<std::vector<double> > motor_data;
};


class acquisition_thread : public RateThread
{
    private:
        std::string moduleName;
        std::string robotName;
        std::string PathName;

        std::vector<dataset_entry> dataset_vect;

        std::string emotionOutPortName;
        std::string imagePortName;
        std::string handcontrolPortName;
        std::string AREPortName;
        std::string rpcMemoryName;

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;
        BufferedPort<Bottle> emotionOutPort;
        BufferedPort<Bottle> handcontrolPort;

        RpcClient AREPort;
        RpcClient rpcMemory;

        Bottle *inputBottle;

        Bottle cmd;
        Bottle reply;

        std::string remotePortsright;
        std::string remotePortsleft;
        std::string remotePortstorso;
        std::string remotePortshead;

        Property optionsright;
        Property optionsleft;
        Property optionstorso;
        Property optionshead;

        PolyDriver robotDeviceright;
        PolyDriver robotDeviceleft;
        PolyDriver robotDevicetorso;
        PolyDriver robotDevicehead;

        IPositionControl *pos_right;
        IPositionControl *pos_left;
        IPositionControl *pos_torso;
        IPositionControl *pos_head;
        IEncoders *encs_right;
        IEncoders *encs_left;
        IEncoders *encs_torso;
        IEncoders *encs_head;

        Vector encoders_right;
        Vector encoders_left;
        Vector encoders_torso;
        Vector encoders_head;
        Vector command_right;
        Vector command_left;
        Vector command_torso;
        Vector command_head;
        Vector tmp_right;
        Vector tmp_left;
        Vector tmp_torso;
        Vector tmp_head;

        // files
        ifstream dataFileIn;
        ifstream motorFileIn;
        ofstream dataFile;
        ofstream dataFilecfg;

    public:

        bool closing;
        int action_counter;

        acquisition_thread(const string &_moduleName, const double _period, const string &_PathName, const string &_robotName);

        virtual bool threadInit();
        virtual void run();
        virtual bool interrupt();
        virtual bool close();
        virtual void threadRelease();

        Bottle get3D(const string &objName);
        Bottle getMemoryBottle(const string &obj);

        // IDL functions
        bool openPorts();
        bool connectRobot();                
        bool continueFromLast();
        bool get3DPos(string obj_label);
        bool storeCommand(std::string command);    
        bool recordAction(string limb);        
        bool recordEmotion();    
        bool recordActionHead();
        bool saveFile();
        bool increaseAction();
        bool resizeDataset();
        bool setEmotion(string emotion);
        bool closeHand(string hand_side);
        bool openHand(string hand_side);
        bool pointHand(string hand_side);
        bool karateHand(string hand_side);
        std::string getSpeech();                          
        virtual bool quit();
};

#endif
