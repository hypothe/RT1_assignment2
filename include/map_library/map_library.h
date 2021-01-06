#ifndef MAP_LIBRARY_H
#define MAP_LIBRARY_H

#include "ros/ros.h"

std::map<std::string, int>::iterator next_map_element(std::map<std::string, int> map, std::string current_key);

#endif
