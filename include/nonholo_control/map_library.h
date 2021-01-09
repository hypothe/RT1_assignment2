#ifndef MAP_LIBRARY_H
#define MAP_LIBRARY_H

#include "ros/ros.h"


/*********************************************//**
* Function returning the element in a map next to
* a key passed, treating the map as circular.
*
* \param map (std::map<std::string, int>):
*           map to iterate on;
* \param current_key (std::string):
*           key to evaluate the next element of;
*
* \retval next_map_element 
*      (std::map<std::string, int>::iterator):
*           iterator containing the pair 
*           <key, value> of the successive 
*           element to the key passed;
*
************************************************/
std::map<std::string, int>::iterator nextMapElement(std::map<std::string, int> map, std::string current_key);

#endif
