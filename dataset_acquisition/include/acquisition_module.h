/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef __ACQUISITION_MODULE_H__
#define __ACQUISITION_MODULE_H__

#include "acquisition_thread.h"

#include "acquisition_IDLserver.h"

class acquisition_module : public RFModule, public acquisition_IDLserver
{
    private:
        // module parameters
        std::string moduleName;
        std::string PathName;
        std::string robotName;

        string handlerPortName;
        RpcServer handlerPort;
        bool closing;
        bool confirm_everything;

        // pointer to a new thread
        acquisition_thread *thread;

        // thread stuff
        double threadPeriod;


        std::vector<dataset_entry> dataset_vect;

        std::string speechInPortName;

        BufferedPort<Bottle> speechInPort;

        Bottle *inputBottle;


    public:

        virtual bool configure(ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();
        virtual bool updateModule();
        virtual double getPeriod();

        // IDL functions
        bool attach(yarp::os::RpcServer &source);
        bool openPorts();
        std::string getSpeech();                       
        virtual bool quit();
};

#endif
