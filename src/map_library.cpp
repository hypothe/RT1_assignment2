
#include "map_library/map_library.h"

std::map<std::string, int>::iterator next_map_element(std::map<std::string, int> map, std::string current_key){
    std::map<std::string, int>::iterator it, next;
    if((it = map.find(current_key)) == map.end()){
        ROS_ERROR("Planning algorithm not found in paramater file map description");
        return map.end();
    }
    next = it==--map.end()?map.begin():++it;
    return next;
}

