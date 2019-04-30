/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "acquisition_thread.h"

acquisition_thread::acquisition_thread(const string &_moduleName, const double _period, const string &_PathName, const string &_robotName):moduleName(_moduleName),RateThread(int(_period*1000.0)),PathName(_PathName),robotName(_robotName)
{
}

bool acquisition_thread::threadInit()
{
    action_counter = 0; // this variable indicates the number of actions recorded
    // create first instance of the dataset vector - check _thread.h for more details on the sturcture
    dataset_vect.resize(1); 
    dataset_vect[0].Pos3D.resize(3);
    dataset_vect[0].one_hot_labels.resize(NUM_OBJECTS);
    dataset_vect[0].motor_data.resize(NUM_STEPS, std::vector<double>(NUM_INPUTS));
    
    bool ok = openPorts();
    ok && connectRobot();
    if(!ok)
    {
        return false;
    }

    closing = false;

    return true;
}

// This function runs in a loop with period
// we use this function for hand controls during teaching - to close, open, point, etc
void acquisition_thread::run()
{
    string command;
    while (!closing)
    {
        inputBottle = handcontrolPort.read(false);
        if (inputBottle != NULL)
        {
            command = inputBottle->toString();
            if(command == "close left")
            {
                closeHand("left");
            }
            if(command == "close right")
            {
                closeHand("right");
            }
            if(command == "open left")
            {
                openHand("left");
            }
            if(command == "open right")
            {
                openHand("right");
            }
            if(command == "finger left")
            {
                pointHand("left");
            }
            if(command == "finger right")
            {
                pointHand("right");
            }
            if(command == "karate left")
            {
                karateHand("left");
            }
            if(command == "karate right")
            {
                karateHand("right");
            }
        }
        Time::delay(0.05);
    }
    return;
}

// Function that opens all ports to the module
bool acquisition_thread::openPorts()
{
    // Port that will receive speech commands to control the hands
    handcontrolPortName = "/"+ moduleName + "/hand_ctrl:i";

    // Port that will control iCub emotions (eyelids, leds for eyebrows and mouth)
    emotionOutPortName = "/"+ moduleName + "/emotion:o";

    // Port that will connect to the cameras to receive images
    imagePortName = "/"+ moduleName + "/image:i";

    // Port that will communicate with actionsRenderingEngine - for teaching and hand controls
    AREPortName = "/" + moduleName + "/are_rpc:o";

    // Port that will communicate with objectPropertyCollector - for labels and object positions
    rpcMemoryName = "/" + moduleName + "/mem_rpc:o";

    bool ret = imagePort.open(imagePortName);
    ret = ret && handcontrolPort.open(handcontrolPortName);
    ret = ret && emotionOutPort.open(emotionOutPortName);
    ret = ret && AREPort.open(AREPortName);
    ret = ret && rpcMemory.open(rpcMemoryName);

    return ret;
}

// This function does all the connections with the robot
// We will have to open the drivers for the force sensors here
bool acquisition_thread::connectRobot()
{
    remotePortsright = "/" + robotName + "/right_arm";
    remotePortsleft = "/" + robotName + "/left_arm";
    remotePortstorso = "/" + robotName + "/torso";
    remotePortshead = "/" + robotName + "/head";

    std::string localPortsright = "/client/right";
    std::string localPortsleft = "/client/left";
    std::string localPortstorso = "/client/torso";
    std::string localPortshead = "/client/head";

    optionsright.put("device", "remote_controlboard");
    optionsright.put("local", localPortsright.c_str());
    optionsright.put("remote", remotePortsright.c_str());

    optionsleft.put("device", "remote_controlboard");
    optionsleft.put("local", localPortsleft.c_str());
    optionsleft.put("remote", remotePortsleft.c_str());

    optionstorso.put("device", "remote_controlboard");
    optionstorso.put("local", localPortstorso.c_str());
    optionstorso.put("remote", remotePortstorso.c_str());

    optionshead.put("device", "remote_controlboard");
    optionshead.put("local", localPortshead.c_str());
    optionshead.put("remote", remotePortshead.c_str());

    bool ok=false;
    double t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (robotDeviceright.open(optionsright))
        {
            ok=true;
            break;
        }
        
        Time::delay(1.0);
    }
    if (!ok)
    {
        yError()<<"Unable to open the right arm Cartesian Controller";
        return false;
    }
	yInfo() << "right arm driver opened successfuly";

    ok=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (robotDeviceleft.open(optionsleft))
        {
            ok=true;
            break;
        }
        
        Time::delay(1.0);
    }
    if (!ok)
    {
        yError()<<"Unable to open the left arm Cartesian Controller";
        robotDeviceright.close();
        return false;
    }
	yInfo() << "left arm driver opened successfuly";

    ok=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (robotDevicetorso.open(optionstorso))
        {
            ok=true;
            break;
        }
        
        Time::delay(1.0);
    }
    if (!ok)
    {
        yError()<<"Unable to open the torso Cartesian Controller";
        robotDeviceright.close();
        robotDeviceleft.close();
        return false;
    }
	yInfo() << "torso driver opened successfuly";

    ok=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (robotDevicehead.open(optionshead))
        {
            ok=true;
            break;
        }
        
        Time::delay(1.0);
    }
    if (!ok)
    {
        yError()<<"Unable to open the head Cartesian Controller";
        robotDeviceright.close();
        robotDeviceleft.close();
        robotDevicetorso.close();
        return false;
    }
	yInfo() << "head driver opened successfuly";

    ok = robotDeviceright.view(pos_right);
    ok = ok && robotDeviceright.view(encs_right);
    ok = ok && robotDeviceleft.view(pos_left);
    ok = ok && robotDeviceleft.view(encs_left);
    ok = ok && robotDevicetorso.view(pos_torso);
    ok = ok && robotDevicetorso.view(encs_torso);
    ok = ok && robotDevicehead.view(pos_head);
    ok = ok && robotDevicehead.view(encs_head);

    if(!ok)
    {
        yError("Problems acquiring interfaces\n");
        return false;
    }

    int nj_right=0, nj_left=0, nj_torso=0, nj_head=0;
    pos_right->getAxes(&nj_right);
    pos_left->getAxes(&nj_left);
    pos_torso->getAxes(&nj_torso);
    pos_head->getAxes(&nj_head);
    encoders_right.resize(nj_right);
    encoders_left.resize(nj_left);
    encoders_torso.resize(nj_torso);
    encoders_head.resize(nj_head);
    tmp_right.resize(nj_right);
    tmp_left.resize(nj_left);
    tmp_torso.resize(nj_torso);
    tmp_head.resize(nj_head);
    command_right.resize(nj_right);
    command_left.resize(nj_left);
    command_torso.resize(nj_torso);
    command_head.resize(nj_head);

    yInfo("waiting for encoders");
    while(!encs_left->getEncoders(encoders_left.data()) && !closing)
    {
        Time::delay(0.1);
    }
    Time::delay(0.1);
    yInfo("waiting for encoders");
    while(!encs_right->getEncoders(encoders_right.data()) && !closing)
    {
        Time::delay(0.1);
    }
    Time::delay(0.1);
    yInfo("waiting for encoders");
    while(!encs_torso->getEncoders(encoders_torso.data()) && !closing)
    {
        Time::delay(0.1);
    }
    Time::delay(0.1);
    yInfo("waiting for encoders");
    while(!encs_head->getEncoders(encoders_head.data()) && !closing)
    {
        Time::delay(0.1);
    }
    return true;
}

bool acquisition_thread::interrupt()
{
    imagePort.interrupt();
    rpcMemory.interrupt();
    emotionOutPort.interrupt();
    AREPort.interrupt();
    handcontrolPort.interrupt();

    return true;
}

bool acquisition_thread::close()
{
    yInfo("starting shutdown procedure");
    closing = true;
    imagePort.close();
    rpcMemory.close();
    emotionOutPort.close();
    AREPort.close();
    handcontrolPort.close();

    robotDeviceright.close();
    robotDeviceleft.close();
    robotDevicetorso.close();
    robotDevicehead.close();

    yInfo("Module terminated");
    Time::delay(0.1);
    return true;
}

// this function will load the data saved on previous files
// allows the module to continue from the last saved step
bool acquisition_thread::continueFromLast()
{
    std::vector<std::string> tmp_vector;
    std::vector<int> label_vector(NUM_OBJECTS);
    std::string tmp_str, image_name, sentence, line;

    // Data is stored in 2 files, a .cfg that saves general information (object name, command, etc)
    // and a .txt file that stores the actual motor data
    // these two files are synced - element 1 in .cfg corresponds to sequence 1 in .txt

    yInfo("Loading from file...\n");
    dataFileIn.open(PathName + "/mtrnnTrain.cfg");
    if(dataFileIn.is_open())
    {
        while(!closing)
        {
            getline(dataFileIn, line);
            if(dataFileIn.eof())
            {
                break;
            }
            // a new element/sequence starts with "ELEMENT #", with # being the number
            /* it has the following structure:
                ELEMENT #
                Image: image_file_name.jpg
                Sentence: command_string
                Label vector:	0	1	0	0	0	0	0	0	0	
            */

            // IMPORTANT -  if this structure changes, this function has to be adapted
            if(line.find("ELEMENT") != std::string::npos)
            {
                getline(dataFileIn, line);
                tmp_vector = split(line, ':');
                tmp_str = tmp_vector[1];
                if(tmp_str.find(" ") != std::string::npos) tmp_str.replace(tmp_str.find(" "), 1, "");
                if(tmp_str.find("\r") != std::string::npos) tmp_str.replace(tmp_str.find("\r"), 1, "");
                if(tmp_str.find("\n") != std::string::npos) tmp_str.replace(tmp_str.find("\n"), 1, "");
                image_name = tmp_str;

                getline(dataFileIn, line);
                tmp_vector = split(line, ':');
                tmp_str = tmp_vector[1];
                if(tmp_str.find(" ") != std::string::npos) tmp_str.replace(tmp_str.find(" "), 1, "");
                if(tmp_str.find("\r") != std::string::npos) tmp_str.replace(tmp_str.find("\r"), 1, "");
                if(tmp_str.find("\n") != std::string::npos) tmp_str.replace(tmp_str.find("\n"), 1, "");
                sentence = tmp_str;
                
                getline(dataFileIn, line);
                tmp_vector = split(line, '\t');
                for(int k = 1; k<=NUM_OBJECTS; k++)
                {
                    tmp_str = tmp_vector[k];
                    if(tmp_str.find("\r") != std::string::npos) tmp_str.replace(tmp_str.find("\r"),1,"");
                    if(tmp_str.find("\n") != std::string::npos) tmp_str.replace(tmp_str.find("\n"),1,"");
                    label_vector[k-1] = stoi(tmp_str);
                }

                dataset_vect[action_counter].img_name = image_name;
                dataset_vect[action_counter].sentence = sentence;
                for(int p = 0; p<NUM_OBJECTS; p++)
                {
                    dataset_vect[action_counter].one_hot_labels[p] = label_vector[p];
                }
                action_counter++;
                dataset_vect.resize(action_counter+1);
                dataset_vect[action_counter].Pos3D.resize(3);
                dataset_vect[action_counter].one_hot_labels.resize(NUM_OBJECTS);
                dataset_vect[action_counter].motor_data.resize(NUM_STEPS, std::vector<double>(NUM_INPUTS));
            }
        }
        dataFileIn.close();
        // now we load from the .txt file, with all the motor data
        motorFileIn.open(PathName + "/mtrnnTD.txt");
        if(motorFileIn.is_open())
        {
            int file_counter = 0;
            while(!closing)
            {
                getline(motorFileIn, line);
                if(motorFileIn.eof())
                {
                    break;
                }
                // the structure here is simpler, it starts with SEQUENCE #, and is followed
                // by NUM_STEPS lines, each with NUM_ENCODERS values
                if(line.find("SEQUENCE") != std::string::npos)
                {
                    for(int i = 0; i <100; i++)
                    {
                        getline(motorFileIn, line);
                        tmp_vector = split(line, '\t');
                        int k = 0;
                        for(k = 0; k<NUM_ENCODERS; k++)
                        {
                            dataset_vect[file_counter].motor_data[i][k] = stof(tmp_vector[k], nullptr);
                        }
                        // load the position into the matrix
                        dataset_vect[file_counter].motor_data[i][k] = stof(tmp_vector[k], nullptr);
                        dataset_vect[file_counter].motor_data[i][k+1] = stof(tmp_vector[k+1], nullptr);
                        dataset_vect[file_counter].motor_data[i][k+2] = stof(tmp_vector[k+2], nullptr);
                        // and to the file itself (duplicated)
                        dataset_vect[file_counter].Pos3D[0] = stof(tmp_vector[k]);
                        dataset_vect[file_counter].Pos3D[1] = stof(tmp_vector[k+1]);
                        dataset_vect[file_counter].Pos3D[2] = stof(tmp_vector[k+2]);

                        // and now we save the emotion "encoders"
                        dataset_vect[file_counter].motor_data[i][k+3] = stof(tmp_vector[k+3], nullptr);
                        tmp_str = tmp_vector[k+4];
                        if(tmp_str.find("\r") != std::string::npos) tmp_str.replace(tmp_str.find("\r"),1,"");
                        if(tmp_str.find("\n") != std::string::npos) tmp_str.replace(tmp_str.find("\n"),1,"");   
                        dataset_vect[file_counter].motor_data[i][k+4] = stof(tmp_str, nullptr);                     
                    }
                    file_counter++;
                }
            }
        }
        else
        {
            yError("File not found, exiting\n");
            return false;
        }
        motorFileIn.close();
    }
    else
    {
        yError("File not found, exiting\n");
        return false;
    }
    return true;
}

// copied from Poeticon++/activityInterface - for communication with OPC
Bottle acquisition_thread::getMemoryBottle(const string &obj)
{
    Bottle memoryReply;
    Bottle cmdMemory,replyMemory,replyMemoryProp;
    cmdMemory.addVocab(Vocab::encode("ask"));
    Bottle &cont=cmdMemory.addList().addList();
    cont.addString("name");
    cont.addString("==");
    cont.addString(obj);
    rpcMemory.write(cmdMemory,replyMemory);
    
    if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
    {
        if (Bottle *idField=replyMemory.get(1).asList())
        {
            if (Bottle *idValues=idField->get(1).asList())
            {
                int id=idValues->get(0).asInt();
                
                Bottle cmdTime;Bottle cmdReply;
                cmdTime.addVocab(Vocab::encode("time"));
                Bottle &contmp=cmdTime.addList();
                Bottle &list_tmp=contmp.addList();
                list_tmp.addString("id");
                list_tmp.addInt(id);
                rpcMemory.write(cmdTime,cmdReply);
                
                Bottle *timePassed = cmdReply.get(1).asList();
                double time = timePassed->get(0).asDouble();
                
                if (time < 1.0)
                {
                    cmdMemory.clear();
                    cmdMemory.addVocab(Vocab::encode("get"));
                    Bottle &content=cmdMemory.addList();
                    Bottle &list_bid=content.addList();
                    list_bid.addString("id");
                    list_bid.addInt(id);
                    rpcMemory.write(cmdMemory,replyMemoryProp);
                    
                    memoryReply.addList() = *replyMemoryProp.get(1).asList();
                }
            }
        }
    }
    return memoryReply;
}

// copied from Poeticon++/activityInterface - for communication with OPC
Bottle acquisition_thread::get3D(const string &objName)
{
    Bottle Memory = getMemoryBottle(objName);
    Bottle position3D;
    
    if (Bottle *propField = Memory.get(0).asList())
    {
        if (propField->check("position_3d"))
        {
            Bottle *propFieldPos = propField->find("position_3d").asList();
            
            for (int i=0; i < propFieldPos->size(); i++)
            {
                position3D.addDouble(propFieldPos->get(i).asDouble());
            }
        }
    }
    return position3D;
}

// function that obtains the 3D position of the object (through OPC)
// stores the position on the corresponding data slot
bool acquisition_thread::get3DPos(string obj_label)
{
    yInfo("waiting for position bottle input... \n");

    Bottle position = get3D(obj_label);
    yInfo("Position Bottle: %s", position.toString().c_str());

    dataset_vect[action_counter].Pos3D[0] = position.get(0).asDouble();
    dataset_vect[action_counter].Pos3D[1] = position.get(1).asDouble();
    dataset_vect[action_counter].Pos3D[2] = position.get(2).asDouble();
    for(int i = 0; i<NUM_STEPS; i++)
    {
        dataset_vect[action_counter].motor_data[i][NUM_ENCODERS] = dataset_vect[action_counter].Pos3D[0];
        dataset_vect[action_counter].motor_data[i][NUM_ENCODERS+1] = dataset_vect[action_counter].Pos3D[1];
        dataset_vect[action_counter].motor_data[i][NUM_ENCODERS+2] = dataset_vect[action_counter].Pos3D[2];
    }

    // This is the list of objects we were considering - it can be changed
    // IMPORTANT - if the NUM_OBJECTS changes, you need to change it here too
    if(obj_label == "cube") dataset_vect[action_counter].one_hot_labels[0] = 1.0;
    if(obj_label == "box") dataset_vect[action_counter].one_hot_labels[1] = 1.0;
    if(obj_label == "ball") dataset_vect[action_counter].one_hot_labels[2] = 1.0;
    if(obj_label == "car") dataset_vect[action_counter].one_hot_labels[3] = 1.0;
    if(obj_label == "bottle") dataset_vect[action_counter].one_hot_labels[4] = 1.0;
    if(obj_label == "dog") dataset_vect[action_counter].one_hot_labels[5] = 1.0;
    if(obj_label == "carrots") dataset_vect[action_counter].one_hot_labels[6] = 1.0;
    if(obj_label == "bear") dataset_vect[action_counter].one_hot_labels[7] = 1.0;
    if(obj_label == "fish") dataset_vect[action_counter].one_hot_labels[8] = 1.0;
    if(obj_label == "tool") dataset_vect[action_counter].one_hot_labels[9] = 1.0;
    if(obj_label == "rake") dataset_vect[action_counter].one_hot_labels[10] = 1.0;
    if(obj_label == "fruit") dataset_vect[action_counter].one_hot_labels[11] = 1.0;
    if(obj_label == "cat") dataset_vect[action_counter].one_hot_labels[12] = 1.0;
    if(obj_label == "cereals") dataset_vect[action_counter].one_hot_labels[13] = 1.0;
    if(obj_label == "turtle") dataset_vect[action_counter].one_hot_labels[14] = 1.0;
    if(obj_label == "lego") dataset_vect[action_counter].one_hot_labels[15] = 1.0;
    if(obj_label == "octopus") dataset_vect[action_counter].one_hot_labels[16] = 1.0;
    if(obj_label == "spoon") dataset_vect[action_counter].one_hot_labels[17] = 1.0;


    // for now the images are saved under the name "image-#.jpg".
    dataset_vect[action_counter].img_name = "image-" + to_string(action_counter) + ".jpg";

    // now we store the image from the iCub left camera
    // apparently this is old implementation - I don't know the new one though....
    while(!closing)
    {
        ImageOf<PixelRgb> *img = imagePort.read(false);
        if(img == NULL) continue;
        Mat cv_image = cvarrToMat((IplImage *)img->getIplImage());
        imwrite(dataset_vect[action_counter].img_name, cv_image);
        break;
    }
    return true;
}

// function that stores the command string on the dataset structure
bool acquisition_thread::storeCommand(string command)
{
    dataset_vect[action_counter].sentence = command;
    return true;
}

// function that increases the action index by 1, "jumping" to the next action to collect
bool acquisition_thread::increaseAction()
{
    action_counter++;
    return true;
}

// this function resizes the dataset vector to create a new dataset class instance to acomodate the next data collection
bool acquisition_thread::resizeDataset()
{
    dataset_vect.resize(action_counter+1);
    dataset_vect[action_counter].Pos3D.resize(3);
    dataset_vect[action_counter].one_hot_labels.resize(NUM_OBJECTS);
    dataset_vect[action_counter].motor_data.resize(NUM_STEPS, std::vector<double>(NUM_INPUTS));
    return true;
}

// function that sets the emotion on the iCub robot depending on the input string
bool acquisition_thread::setEmotion(string emotion)
{
    if(emotion == "smile")
    {
        Bottle &emotionBottle = emotionOutPort.prepare();
        emotionBottle.clear();
        emotionBottle.addString("set");
        emotionBottle.addString("leb");
        emotionBottle.addString("hap");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle1 = emotionOutPort.prepare();
        emotionBottle1.clear();
        emotionBottle1.addString("set");
        emotionBottle1.addString("reb");
        emotionBottle1.addString("hap");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle2 = emotionOutPort.prepare();
        emotionBottle2.clear();
        emotionBottle2.addString("set");
        emotionBottle2.addString("mou");
        emotionBottle2.addString("hap");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle3 = emotionOutPort.prepare();
        emotionBottle3.clear();
        emotionBottle3.addString("set");
        emotionBottle3.addString("eli");
        emotionBottle3.addString("hap");
        emotionOutPort.write();

        // this encoding can (should?) be changed
        for(int i=0; i<NUM_STEPS; i++)
        {
            dataset_vect[action_counter].motor_data[i][44] = 1.0; // valence
            dataset_vect[action_counter].motor_data[i][45] = 0.5; // amplitude
        }
    }
    if(emotion == "cry")
    {
        Bottle &emotionBottle = emotionOutPort.prepare();
        emotionBottle.clear();
        emotionBottle.addString("set");
        emotionBottle.addString("leb");
        emotionBottle.addString("sad");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle1 = emotionOutPort.prepare();
        emotionBottle1.clear();
        emotionBottle1.addString("set");
        emotionBottle1.addString("reb");
        emotionBottle1.addString("sad");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle2 = emotionOutPort.prepare();
        emotionBottle2.clear();
        emotionBottle2.addString("set");
        emotionBottle2.addString("mou");
        emotionBottle2.addString("sad");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle3 = emotionOutPort.prepare();
        emotionBottle3.clear();
        emotionBottle3.addString("set");
        emotionBottle3.addString("eli");
        emotionBottle3.addString("sad");
        emotionOutPort.write();

        // this encoding can (should?) be changed
        for(int i=0; i<NUM_STEPS; i++)
        {
            dataset_vect[action_counter].motor_data[i][44] = 0.5; // valence
            dataset_vect[action_counter].motor_data[i][45] = 0.5; // amplitude
        }
    }
    if(emotion == "frown")
    {
        Bottle &emotionBottle = emotionOutPort.prepare();
        emotionBottle.clear();
        emotionBottle.addString("set");
        emotionBottle.addString("leb");
        emotionBottle.addString("ang");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle1 = emotionOutPort.prepare();
        emotionBottle1.clear();
        emotionBottle1.addString("set");
        emotionBottle1.addString("reb");
        emotionBottle1.addString("ang");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle2 = emotionOutPort.prepare();
        emotionBottle2.clear();
        emotionBottle2.addString("set");
        emotionBottle2.addString("mou");
        emotionBottle2.addString("neu");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle3 = emotionOutPort.prepare();
        emotionBottle3.clear();
        emotionBottle3.addString("set");
        emotionBottle3.addString("eli");
        emotionBottle3.addString("ang");
        emotionOutPort.write();

        // this encoding can (should?) be changed
        for(int i=0; i<NUM_STEPS; i++)
        {
            dataset_vect[action_counter].motor_data[i][44] = 0.5; // valence
            dataset_vect[action_counter].motor_data[i][45] = 1.0; // amplitude
        }
    }
    if(emotion == "laugh")
    {
        Bottle &emotionBottle = emotionOutPort.prepare();
        emotionBottle.clear();
        emotionBottle.addString("set");
        emotionBottle.addString("leb");
        emotionBottle.addString("sur");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle1 = emotionOutPort.prepare();
        emotionBottle1.clear();
        emotionBottle1.addString("set");
        emotionBottle1.addString("reb");
        emotionBottle1.addString("sur");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle2 = emotionOutPort.prepare();
        emotionBottle2.clear();
        emotionBottle2.addString("set");
        emotionBottle2.addString("mou");
        emotionBottle2.addString("hap");
        emotionOutPort.write();

        Time::delay(0.1);

        Bottle &emotionBottle3 = emotionOutPort.prepare();
        emotionBottle3.clear();
        emotionBottle3.addString("set");
        emotionBottle3.addString("eli");
        emotionBottle3.addString("sur");
        emotionOutPort.write();

        // this encoding can (should?) be changed
        for(int i=0; i<NUM_STEPS; i++)
        {
            dataset_vect[action_counter].motor_data[i][44] = 1.0; // valence
            dataset_vect[action_counter].motor_data[i][45] = 1.0; // amplitude
        }
    }
    return true;
}    

// this function records and action for an emotion "action"
// basically it is faster than a normal recording - it doesn't go through all the 100 steps
bool acquisition_thread::recordEmotion()
{
    bool ret= encs_head->getEncoders(encoders_head.data());
    ret = ret && encs_torso->getEncoders(encoders_torso.data());
    ret = ret && encs_right->getEncoders(encoders_right.data());
    ret = ret && encs_left->getEncoders(encoders_left.data());
     
    if (!ret)
    {
        yError("Error receiving encoders, check connectivity with the robot\n");
        return false;
    }
    command_right = encoders_right;
    command_left = encoders_left;
    command_torso = encoders_torso;
    command_head = encoders_head;

    for(int i = 0; i<NUM_STEPS; i++)
    {
        //here we do the normalization of the encoder values
        dataset_vect[action_counter].motor_data[i][0] = (command_head[0]+39.9) / 69.30;
        dataset_vect[action_counter].motor_data[i][1] = (command_head[1]+68.9)/ 128.7;
        dataset_vect[action_counter].motor_data[i][2] = (command_head[2]+55)/110;
        dataset_vect[action_counter].motor_data[i][3] = (command_head[3]+35)/50;
        dataset_vect[action_counter].motor_data[i][4] = (command_head[4]+49.98)/100.98;
        dataset_vect[action_counter].motor_data[i][5] = (command_head[5])/90;
        dataset_vect[action_counter].motor_data[i][6] = (command_torso[0]+50)/100;
        dataset_vect[action_counter].motor_data[i][7] = (command_torso[1]+30)/60;
        dataset_vect[action_counter].motor_data[i][8] = (command_torso[2]+9.6)/79.2;
        dataset_vect[action_counter].motor_data[i][9] = (command_left[0]+94.5)/104;
        dataset_vect[action_counter].motor_data[i][10] = (command_left[1])/160.8;
        dataset_vect[action_counter].motor_data[i][11] = (command_left[2]+36.27)/115.83;
        dataset_vect[action_counter].motor_data[i][12] = (command_left[3]-20.385)/90.5;
        dataset_vect[action_counter].motor_data[i][13] = (command_left[4]+90)/180;
        dataset_vect[action_counter].motor_data[i][14] = (command_left[5]+90)/90;
        dataset_vect[action_counter].motor_data[i][15] = (command_left[6]+15.8)/59.4;
        dataset_vect[action_counter].motor_data[i][16] = (command_left[7])/60;
        dataset_vect[action_counter].motor_data[i][17] = (command_left[8]-15)/79.6;
        dataset_vect[action_counter].motor_data[i][18] = (command_left[9])/90;
        dataset_vect[action_counter].motor_data[i][19] = (command_left[10])/180;
        dataset_vect[action_counter].motor_data[i][20] = (command_left[11])/90;
        dataset_vect[action_counter].motor_data[i][21] = (command_left[12])/180;
        dataset_vect[action_counter].motor_data[i][22] = (command_left[13])/90;
        dataset_vect[action_counter].motor_data[i][23] = (command_left[14])/180;
        dataset_vect[action_counter].motor_data[i][24] = (command_left[15])/270;
        dataset_vect[action_counter].motor_data[i][25] = (command_right[0]+94.5)/104;
        dataset_vect[action_counter].motor_data[i][26] = (command_right[1])/160;
        dataset_vect[action_counter].motor_data[i][27] = (command_right[2]+36.27)/115.83;
        dataset_vect[action_counter].motor_data[i][28] = (command_right[3]-20.385)/90.5;
        dataset_vect[action_counter].motor_data[i][29] = (command_right[4]+90)/180;
        dataset_vect[action_counter].motor_data[i][30] = (command_right[5]+90)/90;
        dataset_vect[action_counter].motor_data[i][31] = (command_right[6]+15.8)/59.4;
        dataset_vect[action_counter].motor_data[i][32] = (command_right[7])/60;
        dataset_vect[action_counter].motor_data[i][33] = (command_right[8]-15)/79.6;
        dataset_vect[action_counter].motor_data[i][34] = (command_right[9])/90;
        dataset_vect[action_counter].motor_data[i][35] = (command_right[10])/180;
        dataset_vect[action_counter].motor_data[i][36] = (command_right[11])/90;
        dataset_vect[action_counter].motor_data[i][37] = (command_right[12])/180;
        dataset_vect[action_counter].motor_data[i][38] = (command_right[13])/90;
        dataset_vect[action_counter].motor_data[i][39] = (command_right[14])/180;
        dataset_vect[action_counter].motor_data[i][40] = (command_right[15])/270;
    }
    return true;
}

// this action records the motion of the robot
// it loops for NUM_STEPS, with each step being 0.16 seconds 
// TODO: make the step size a macro value too, that can be changed
bool acquisition_thread::recordAction(string limb)
{
    int t = 3;


    // HERE WE SEND THE COMMAND TO ARE TO START TEACHING
    cmd.clear();
    cmd.addString("teach");
    string cmd_string = "action_" + to_string(action_counter);
    cmd.addString(cmd_string);
    cmd.addString("start");
    cmd.addString(limb);
    AREPort.write(cmd, reply);
    yInfo("%s", reply.get(0).asString().c_str());
    yInfo("%i", reply.size());
    if (strcmp(reply.toString().c_str(), "[nack]") == 0)
    {
        yError("failed to start ARE");
        return false;
    }

    // I have this small loop for the person to get ready - with 3 steps
    yInfo("recording will start in %i\n", t);
    while(t > 0)
    {
        Time::delay(0.2);
        t -= 1;
        yInfo("%i...\n", t);
    }
    yInfo("recording now\n");       
 
    int i = 0;
    double time1 = Time::now() + 0.16; // each "step" will be 0.16 seconds
    double time0 = Time::now();
    while(i<NUM_STEPS)
    {
        time0 = Time::now(); // it is better this way since it also takes into account computation time
        if(time0 - time1 >= 0.16) // this ensures all steps will be similarly timed
        {
            yInfo("recording step %i\n",i);
            time1 = Time::now();
            //immediatelly store time, we need to make the windows as close to 0.16 as possible
            bool ret= encs_head->getEncoders(encoders_head.data());
            ret = ret && encs_torso->getEncoders(encoders_torso.data());
            ret = ret && encs_right->getEncoders(encoders_right.data());
            ret = ret && encs_left->getEncoders(encoders_left.data());
             
            if (!ret)
            {
                yError("Error receiving encoders, check connectivity with the robot\n");
                break;
            }
            command_right = encoders_right;
            command_left = encoders_left;
            command_torso = encoders_torso;
            command_head = encoders_head;

            //here we do the normalization of the encoder values
            dataset_vect[action_counter].motor_data[i][0] = (command_head[0]+39.9) / 69.30;
            dataset_vect[action_counter].motor_data[i][1] = (command_head[1]+68.9)/ 128.7;
            dataset_vect[action_counter].motor_data[i][2] = (command_head[2]+55)/110;
            dataset_vect[action_counter].motor_data[i][3] = (command_head[3]+35)/50;
            dataset_vect[action_counter].motor_data[i][4] = (command_head[4]+49.98)/100.98;
            dataset_vect[action_counter].motor_data[i][5] = (command_head[5])/90;
            dataset_vect[action_counter].motor_data[i][6] = (command_torso[0]+50)/100;
            dataset_vect[action_counter].motor_data[i][7] = (command_torso[1]+30)/60;
            dataset_vect[action_counter].motor_data[i][8] = (command_torso[2]+9.6)/79.2;
            dataset_vect[action_counter].motor_data[i][9] = (command_left[0]+94.5)/104;
            dataset_vect[action_counter].motor_data[i][10] = (command_left[1])/160.8;
            dataset_vect[action_counter].motor_data[i][11] = (command_left[2]+36.27)/115.83;
            dataset_vect[action_counter].motor_data[i][12] = (command_left[3]-20.385)/90.5;
            dataset_vect[action_counter].motor_data[i][13] = (command_left[4]+90)/180;
            dataset_vect[action_counter].motor_data[i][14] = (command_left[5]+90)/90;
            dataset_vect[action_counter].motor_data[i][15] = (command_left[6]+15.8)/59.4;
            dataset_vect[action_counter].motor_data[i][16] = (command_left[7])/60;
            dataset_vect[action_counter].motor_data[i][17] = (command_left[8]-15)/79.6;
            dataset_vect[action_counter].motor_data[i][18] = (command_left[9])/90;
            dataset_vect[action_counter].motor_data[i][19] = (command_left[10])/180;
            dataset_vect[action_counter].motor_data[i][20] = (command_left[11])/90;
            dataset_vect[action_counter].motor_data[i][21] = (command_left[12])/180;
            dataset_vect[action_counter].motor_data[i][22] = (command_left[13])/90;
            dataset_vect[action_counter].motor_data[i][23] = (command_left[14])/180;
            dataset_vect[action_counter].motor_data[i][24] = (command_left[15])/270;
            dataset_vect[action_counter].motor_data[i][25] = (command_right[0]+94.5)/104;
            dataset_vect[action_counter].motor_data[i][26] = (command_right[1])/160;
            dataset_vect[action_counter].motor_data[i][27] = (command_right[2]+36.27)/115.83;
            dataset_vect[action_counter].motor_data[i][28] = (command_right[3]-20.385)/90.5;
            dataset_vect[action_counter].motor_data[i][29] = (command_right[4]+90)/180;
            dataset_vect[action_counter].motor_data[i][30] = (command_right[5]+90)/90;
            dataset_vect[action_counter].motor_data[i][31] = (command_right[6]+15.8)/59.4;
            dataset_vect[action_counter].motor_data[i][32] = (command_right[7])/60;
            dataset_vect[action_counter].motor_data[i][33] = (command_right[8]-15)/79.6;
            dataset_vect[action_counter].motor_data[i][34] = (command_right[9])/90;
            dataset_vect[action_counter].motor_data[i][35] = (command_right[10])/180;
            dataset_vect[action_counter].motor_data[i][36] = (command_right[11])/90;
            dataset_vect[action_counter].motor_data[i][37] = (command_right[12])/180;
            dataset_vect[action_counter].motor_data[i][38] = (command_right[13])/90;
            dataset_vect[action_counter].motor_data[i][39] = (command_right[14])/180;
            dataset_vect[action_counter].motor_data[i][40] = (command_right[15])/270;

            i+=1;
        }
    }

    
    // NOW WE SEND THE COMMAND TO ARE TO STOP TEACHING
    cmd.clear();
    cmd.addString("teach");
    cmd.addString(cmd_string);
    cmd.addString("stop");
    cmd.addString(limb);
    AREPort.write(cmd, reply);
    if (strcmp(reply.toString().c_str(), "[nack]") == 0)
    {
        yError("failed to start ARE");
        return false;
    }
    Time::delay(0.1);
    // AND THEN WE SEND THE COMMAND TO GO HOME
    cmd.clear();
    cmd.addString("home");
    cmd.addString("all");
    AREPort.write(cmd, reply);
    if (strcmp(reply.toString().c_str(), "[nack]") == 0)
    {
        yError("failed to start ARE");
        return false;
    }


    return true;
}

// this function records an action using only the head, basically it doesn't use ARE teach function
bool acquisition_thread::recordActionHead()
{
    int t = 3;

    yInfo("recording will start in %i\n", t);
    while(t > 0)
    {
        Time::delay(0.2);
        t -= 1;
        yInfo("%i...\n", t);
    }
    yInfo("recording now\n");       
 
    int i = 0;
    double time1 = Time::now() + 0.16; // each "step" will be 0.16 seconds
    double time0 = Time::now();
    while(i<NUM_STEPS)
    {
        time0 = Time::now(); // it is better this way since it also takes into account computation time
        if(time0 - time1 >= 0.16) // this ensures all steps will be similarly timed
        {
            yInfo("recording step %i\n",i);
            time1 = Time::now();
            //immediatelly store time, we need to make the windows as close to 0.16 as possible
            bool ret= encs_head->getEncoders(encoders_head.data());
            ret = ret && encs_torso->getEncoders(encoders_torso.data());
            ret = ret && encs_right->getEncoders(encoders_right.data());
            ret = ret && encs_left->getEncoders(encoders_left.data());
             
            if (!ret)
            {
                yError("Error receiving encoders, check connectivity with the robot\n");
                break;
            }
            command_right = encoders_right;
            command_left = encoders_left;
            command_torso = encoders_torso;
            command_head = encoders_head;

            //here we do the normalization of the encoder values
            dataset_vect[action_counter].motor_data[i][0] = (command_head[0]+39.9) / 69.30;
            dataset_vect[action_counter].motor_data[i][1] = (command_head[1]+68.9)/ 128.7;
            dataset_vect[action_counter].motor_data[i][2] = (command_head[2]+55)/110;
            dataset_vect[action_counter].motor_data[i][3] = (command_head[3]+35)/50;
            dataset_vect[action_counter].motor_data[i][4] = (command_head[4]+49.98)/100.98;
            dataset_vect[action_counter].motor_data[i][5] = (command_head[5])/90;
            dataset_vect[action_counter].motor_data[i][6] = (command_torso[0]+50)/100;
            dataset_vect[action_counter].motor_data[i][7] = (command_torso[1]+30)/60;
            dataset_vect[action_counter].motor_data[i][8] = (command_torso[2]+9.6)/79.2;
            dataset_vect[action_counter].motor_data[i][9] = (command_left[0]+94.5)/104;
            dataset_vect[action_counter].motor_data[i][10] = (command_left[1])/160.8;
            dataset_vect[action_counter].motor_data[i][11] = (command_left[2]+36.27)/115.83;
            dataset_vect[action_counter].motor_data[i][12] = (command_left[3]-20.385)/90.5;
            dataset_vect[action_counter].motor_data[i][13] = (command_left[4]+90)/180;
            dataset_vect[action_counter].motor_data[i][14] = (command_left[5]+90)/90;
            dataset_vect[action_counter].motor_data[i][15] = (command_left[6]+15.8)/59.4;
            dataset_vect[action_counter].motor_data[i][16] = (command_left[7])/60;
            dataset_vect[action_counter].motor_data[i][17] = (command_left[8]-15)/79.6;
            dataset_vect[action_counter].motor_data[i][18] = (command_left[9])/90;
            dataset_vect[action_counter].motor_data[i][19] = (command_left[10])/180;
            dataset_vect[action_counter].motor_data[i][20] = (command_left[11])/90;
            dataset_vect[action_counter].motor_data[i][21] = (command_left[12])/180;
            dataset_vect[action_counter].motor_data[i][22] = (command_left[13])/90;
            dataset_vect[action_counter].motor_data[i][23] = (command_left[14])/180;
            dataset_vect[action_counter].motor_data[i][24] = (command_left[15])/270;
            dataset_vect[action_counter].motor_data[i][25] = (command_right[0]+94.5)/104;
            dataset_vect[action_counter].motor_data[i][26] = (command_right[1])/160;
            dataset_vect[action_counter].motor_data[i][27] = (command_right[2]+36.27)/115.83;
            dataset_vect[action_counter].motor_data[i][28] = (command_right[3]-20.385)/90.5;
            dataset_vect[action_counter].motor_data[i][29] = (command_right[4]+90)/180;
            dataset_vect[action_counter].motor_data[i][30] = (command_right[5]+90)/90;
            dataset_vect[action_counter].motor_data[i][31] = (command_right[6]+15.8)/59.4;
            dataset_vect[action_counter].motor_data[i][32] = (command_right[7])/60;
            dataset_vect[action_counter].motor_data[i][33] = (command_right[8]-15)/79.6;
            dataset_vect[action_counter].motor_data[i][34] = (command_right[9])/90;
            dataset_vect[action_counter].motor_data[i][35] = (command_right[10])/180;
            dataset_vect[action_counter].motor_data[i][36] = (command_right[11])/90;
            dataset_vect[action_counter].motor_data[i][37] = (command_right[12])/180;
            dataset_vect[action_counter].motor_data[i][38] = (command_right[13])/90;
            dataset_vect[action_counter].motor_data[i][39] = (command_right[14])/180;
            dataset_vect[action_counter].motor_data[i][40] = (command_right[15])/270;

            i+=1;
        }
    }

    
    Time::delay(0.1);
    // AND THEN WE SEND THE COMMAND TO GO HOME
    cmd.clear();
    cmd.addString("home");
    cmd.addString("all");
    AREPort.write(cmd, reply);
    if (strcmp(reply.toString().c_str(), "[nack]") == 0)
    {
        yError("failed to start ARE");
        return false;
    }


    return true;
}

// functions for closing the hand
bool acquisition_thread::closeHand(string hand_side)
{
    if(hand_side == "left")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE LEFT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("close_hand");
        cmd.addString("left");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side == "right")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE RIGHT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("close_hand");
        cmd.addString("right");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side != "left" && hand_side != "right") return false;
    return true;
}

// functions for opening the hand
bool acquisition_thread::openHand(string hand_side)
{
    if(hand_side == "left")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE LEFT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("open_hand");
        cmd.addString("left");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side == "right")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE RIGHT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("open_hand");
        cmd.addString("right");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side != "left" && hand_side != "right") return false;
    return true;
}

// functions for pointing a finger
bool acquisition_thread::pointHand(string hand_side)
{
    if(hand_side == "left")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE LEFT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("pointing_hand");
        cmd.addString("left");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side == "right")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE RIGHT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("pointing_hand");
        cmd.addString("right");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side != "left" && hand_side != "right") return false;
    return true;
}

// functions for fixing the hand open and fingers close together
bool acquisition_thread::karateHand(string hand_side)
{
    if(hand_side == "left")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE LEFT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("karate_hand");
        cmd.addString("left");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side == "right")
    {
        // WE SEND THE COMMAND TO ARE TO CLOSE THE RIGHT HAND
        cmd.clear();
        cmd.addString("hand");
        cmd.addString("karate_hand");
        cmd.addString("right");
        AREPort.write(cmd, reply);
        if (strcmp(reply.toString().c_str(), "[nack]") == 0)
        {
            yError("failed to start ARE");
            return false;
        }
    }
    if(hand_side != "left" && hand_side != "right") return false;
    return true;
}

// function to print all the data into files. 
// files are stored in the same directory as the binaries
// There are two files: .txt for the encoder data, .cfg for the rest
bool acquisition_thread::saveFile()
{
    dataFile.open(PathName + "/mtrnnTD.txt");
    if(dataFile.is_open())
    {
        for(int j_new = 0; j_new < action_counter; j_new++)
        {
            dataFile << "SEQUENCE " << j_new << "\r\n";
            for(int i = 0; i < NUM_STEPS; i++)
            {
                for(int k = 0; k < NUM_INPUTS; k++)
                {
                    dataFile << dataset_vect[j_new].motor_data[i][k] << "\t";
                }
                dataFile << "\r\n";
            }
        }
        dataFile << "\r\n"; // this is the termination of the file
    }
    else
    {
        yError("File not found, exiting\n");
        return false;
    }
    dataFile.close();

    dataFilecfg.open(PathName + "/mtrnnTrain.cfg");
    
    if(dataFilecfg.is_open())
    {
        for(int j_new = 0; j_new < action_counter; j_new++)
        {
            dataFilecfg << "ELEMENT " << j_new << "\r\n";
            dataFilecfg << "Image: " << dataset_vect[j_new].img_name << "\r\n";
            dataFilecfg << "Sentence: " << dataset_vect[j_new].sentence << "\r\n";
            dataFilecfg << "Label vector:" << "\t";
            for(int i = 0; i < NUM_OBJECTS; i++)
            {
                dataFilecfg << dataset_vect[j_new].one_hot_labels[i] << "\t";
            }
            dataFilecfg << "\r\n";
        }
        dataFilecfg << "\r\n"; // this is the termination of the file
    }
    else
    {
        yError("File not found, exiting\n");
        return false;
    }
    dataFilecfg.close();
    return true;
}

void acquisition_thread::threadRelease()
{
    cout << "thread terminated" << endl;
}

bool acquisition_thread::quit()
{
    closing = true;
    return true;
}
