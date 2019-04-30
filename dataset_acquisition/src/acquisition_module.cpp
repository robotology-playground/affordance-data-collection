/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "acquisition_module.h"

bool acquisition_module::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("acquisition")).asString();
    setName(moduleName.c_str());

    robotName = rf.check("robot", Value("icubSim")).asString();
    setName(robotName.c_str());
    
    char PathName_arr[FILENAME_MAX]; // FILENAME_MAX defined in stdio.h

    // Pathname will point to the directory where the binaries are 
    // ATTENTION - this is where the dataset files will be printed for now
    getcwd(PathName_arr, FILENAME_MAX);
    PathName = string(PathName_arr); 
    yInfo("path name is: %s", PathName.c_str());

    // Opens the RPC port for the module - for now just for quitting the module
    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);

    // set this flag to true if you want to talk a lot
    // It will ask you to confirm every step, giving you full control of the module
    // at the cost of time spent talking to the robot
    confirm_everything = false;

    // thread stuff
    threadPeriod = 0.033; // [s]

    thread = new acquisition_thread(moduleName,threadPeriod, PathName, robotName);

    // start the thread to do the work
    if (!thread->start())
    {
        delete thread;
        return false;
    }

    if(!openPorts())
    {
        yError("Failed to open speech port");
        return false;
    }
    closing = false;

    // It will ask you to confirm if you want to continue from where you last stopped
    // this is useful if you are running experiments over several runs. 
    // answer "no" if you want to start from scratch
    string speech_str;
    bool ok;
    while(!closing)
    {
        yInfo("Should we continue from last instance?");
        speech_str = getSpeech();
        if(speech_str == "yes")
        {
            ok = thread->continueFromLast();
            if(!ok)
            {
                yError("failed to load");
                thread->close();
                closing = true;
                return false;
            }
            break;
        }
        if(speech_str == "no") break;
    }
    return true;
}

// Function to open the module ports (here the only port that matters is speech)
bool acquisition_module::openPorts()
{
    speechInPortName = "/"+ moduleName + "/speech:i";
    bool ret = speechInPort.open(speechInPortName);
    return ret;
}

bool acquisition_module::interruptModule()
{
    handlerPort.interrupt();
    speechInPort.interrupt();
    thread->askToStop();
    return true;
}

bool acquisition_module::close()
{
    yInfo("starting shutdown procedure");
    thread->interrupt();
    thread->close();
    thread->stop();
    if (thread) delete thread;

    speechInPort.close();
    handlerPort.close();

    yInfo("Module terminated");
    Time::delay(0.1);
    return true;
}

// This function is constantly updating (like an eternal loop)
// It contains the basic steps for the acquisition module, like a state machine
// It is fully controlled by speech - The commands can be sent to the speech port
// Requires a separate speech module (Or a module that writes these commands to the speech port)
bool acquisition_module::updateModule()
{
    bool ok;
    string speech_str;
    // first we get the position of the object, along with its label
    // it is assumed that a separate model will handle this - be it IOL or YOLO 
    yInfo("Please say the name of the object");
    speech_str = getSpeech();

    ok = thread->get3DPos(speech_str);
    if(!ok)
    {
        thread->close();
        quit();
    }

    // now we confirm with the user that everything is fine
    if(confirm_everything)
    {
        yInfo("Is the image ok? y/n \n");
        while (!closing)
        {
            speech_str = getSpeech();
            if(speech_str == "no")
            {
                return !closing;
            }
            if(speech_str == "yes")
            {
                break;
            }
            if(speech_str == "abort")
            {
                thread->close();
                quit();
                return !closing;
            }
        }
    }
    // Now that the picture is good, we can go into the sentence generation.
    // For this, and since we have multiple options, we need to consider another loop 
    string verbal_command;
    while(!closing)
    {
        // we get the command from speech...
        yInfo("Please speak the command to the robot.\n");
        verbal_command = getSpeech();
        yInfo("Command was: %s \n", verbal_command.c_str());
        // we ask the user to confirm the command
        if(confirm_everything)
        {
            yInfo("Is the command ok? y/n \n");
            speech_str = getSpeech();
            if(speech_str == "yes")
            {
                // if it is good, we store it
                thread->storeCommand(verbal_command);
                break;
            }
            if(speech_str == "abort")
            {
                return !closing;
            }
        }
        else
        {
            // we store the command
            thread->storeCommand(verbal_command);
            break;
        }
    }

    // now we set the emotion for the robot
    // for now only these 4 emotions are planned for
    if((verbal_command == "smile") || (verbal_command == "frown") || (verbal_command == "laugh") || (verbal_command == "cry"))
    {
        while(!closing)
        {
            // we confirm the emotion
            if(confirm_everything)
            {
                yInfo("Is the command ok? y/n \n");
                speech_str = getSpeech();
                if(speech_str == "yes")
                {
                    // and we store it
                    thread->setEmotion(verbal_command);
                    thread->recordEmotion();
                    break;
                }
                if(speech_str == "abort")
                {
                    return !closing;
                }
            }
            else
            {
                // we store it
                thread->setEmotion(verbal_command);
                thread->recordEmotion();
                break;
            }
        }
    }
    else
    {
        // if we are not interested in emotions, the default will be set to "smile"
        thread->setEmotion("smile");

        // first we ask the user which limb it will use (for now, left, right, or head)
        string limb;
        while(!closing)
        {
            yInfo("Please say which arm to use, or head.\n");
            limb = getSpeech();
            break;
        }


        // Now we start the recording loop. first we warn the user that it will start, 
        // then we take the encoders each 0.16 seconds, or as close as possible 
        // check the _thread.cpp file for more details
        if(limb == "head")
        {
            yInfo("recording action number %i with head\n", thread->action_counter);
            thread->recordActionHead();
        }
        else
        {
            yInfo("recording action number %i with arm\n", thread->action_counter);
            thread->recordAction(limb);
        }
    }

    // when the motion is over, we confirm with the user if we want to store it
    if(confirm_everything)
    {
        yInfo("do you want to save this motion? y/n\n");
        while(!closing)
        {
            speech_str = getSpeech();
            if(speech_str == "no")
            {
                return !closing;
            }
            if(speech_str == "yes")
            {
                // if yes, we continue to the next action
                thread->increaseAction();
                break;
            }
        }
    }
    else
    {
        // now we continue for the next action
        thread->increaseAction();
    }

    // now we ask if we should save it to a file
    if(confirm_everything)
    {
        yInfo("do you want to save current entries to a file? y/n (default is no)\n");
        while (!closing)
        {
            speech_str = getSpeech();
            if(speech_str == "yes")
            {
                ok = thread->saveFile();
                if(!ok)
                {
                    thread->close();
                    quit();
                    return !closing;
                }
                break;
            }
            if(speech_str == "no")
            {
                break;
            }
        }
    }
    else // if we don't confirm everything, then we save every instance
    {
        ok = thread->saveFile();
    }

    // now we ask if we should continue anyway, or stop recording for now (closes the module)
    yInfo("do you want to continue with data collection? y/n (default is no)\n");
    while (!closing)
    {
        speech_str = getSpeech();
        if(speech_str == "yes")
        {
            thread->resizeDataset();
            break;
        }
        if(speech_str == "no")
        {
            thread->close();
            quit();
            return !closing;
        }
    }
    return !closing;
}

// function that reads speech input from the port (called at several places)
std::string acquisition_module::getSpeech()
{
    while(!closing)
    {
        inputBottle = speechInPort.read(false);
        if (inputBottle != NULL)
        {
            return inputBottle->toString();
        }
        Time::delay(0.1);
    }
    return "";
}

// defines the period of the updateModule() main loop
double acquisition_module::getPeriod()
{
    return 0.05;
}

// function that binds the rpc port to the module (for rpc communication - not really used now)
bool acquisition_module::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool acquisition_module::quit()
{
    closing = true;
    return true;
}

