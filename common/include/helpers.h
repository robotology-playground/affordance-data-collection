/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef __HELPERS_H__
#define __HELPERS_H__

#include <sstream>
#include <vector>
#include <string>

using namespace std;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

int find_element(vector<string> vect, string elem);

std::vector<std::string> split(const std::string &s, char delim);

int vect_compare (vector<string> vect1, vector<string> vect2);

string NumbertoString(int number);

#endif
