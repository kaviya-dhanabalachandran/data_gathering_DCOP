#ifndef _UTILS_H_
#define _UTILS_H_

#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <map>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <iterator>
#include <random>
#include <deque>

using namespace std;

#define a 0.97
#define tau 617.878

class Util
{

public:
	struct Position {
		float x;
		float y;
	};

	struct PositionInt {
			int x;
			int y;
		};

  /* Class constructor. */
    Util();

    /* Class destructor. */
    virtual ~Util() {
    }



    struct Hash {
       size_t operator() (const Position &pos) const {
                                               
         std::string temp = to_string(pos.x) + to_string(pos.y);
                      
         return (temp.length());
      }
    };

  float speed = 0.1; // m/s
  int frequencyOfUpdate = 2; //Every 2 seconds

  inline float findDistance(Position* start, Position* goal)
  {
	  return sqrt(pow(goal->y-start->y,2)+ pow(goal->x-start->x,2));
  }

  inline float findTheta(Position* agentPosition, Position* goalPosition)
  {
	  return atan2(goalPosition->y - agentPosition->y,
	  			goalPosition->x - agentPosition->x);
  }

  inline bool comparePosition(Position* agentPosition, Position* goalPosition)
  {
    return (goalPosition->y == agentPosition->y) && (goalPosition->x == agentPosition->x);
  }
 
 float weightCalculation(uint64_t t);

 std::tuple<uint8_t, float>  findBestAgent(Position relayPos, vector<Position> agentPositionList, vector<uint64_t> timeLastDataCollected);

};

#endif
