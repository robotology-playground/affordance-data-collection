#acquisition.thrift

/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

service acquisition_IDLserver
{
    /**
    * Quit the module.
    * Abruptly terminates the module and the rpc port
    **/
    bool quit();

	/**
	* Waits for speech processing and prints the output sentence
	**/
	string getAction();

	/**
	* Starts listening for an emotion, and sets the icub in the corresponding emotion
	**/
	bool setEmotion();
}
