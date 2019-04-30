/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <yarp/os/Network.h>
#include "acquisition_module.h"

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;

    Property params;
    params.fromCommand(argc, argv);
    if (!params.check("robot"))
    {
        yError("please specify the name of the robot\n");
        yError("--robot name (e.g. icub)\n");
        return 1;
    }

    rf.setVerbose(false);
    rf.configure(argc, argv);

    if(! yarp.checkNetwork() )
    {
        yError("YARP server not available!");
        return 1; // EXIT_FAILURE
    }

    acquisition_module module;
    module.runModule(rf);
    return 0;
}
