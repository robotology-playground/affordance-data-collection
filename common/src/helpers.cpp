/*
 * Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
 * Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
 * 
 * Author: Alexandre Antunes <aleksander88@gmail.com>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "helpers.h"

using namespace std;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

int find_element(vector<string> vect, string elem) {
    int flag_true = 0;
    for (int i = 0; i < vect.size(); ++i){
        if (vect[i] == elem){
            flag_true = 1;
            break;
        }
    }
    return flag_true;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

int vect_compare (vector<string> vect1, vector<string> vect2){
    if (vect1.size() != vect2.size()){
        return 0;
    }
    for (int i = 0; i < vect1.size(); ++i){
        if (find_element(vect2, vect1[i]) == 0){
            return 0;
        }
    }
    return 1;
}

string NumbertoString (int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}
