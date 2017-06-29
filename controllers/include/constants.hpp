#ifndef LIB_CONSTANTS_H
#define LIB_CONSTANTS_H

#include <iostream>
#include <cstdint>
#include <vector>

//using namespace std;
class Constants
{
 public:
 std::vector<int> basestation_id { 1,2,3 };
 std::vector<int> relay_id { 4,5 };
 std::vector<int> mission_agent_id { 6,7,8 };

// Map with id and position of neighbours.
//typedef std::map<uint8_t,std::vector<double> > NeighbourMap;
 
};
#endif