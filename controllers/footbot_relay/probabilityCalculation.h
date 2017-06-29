#ifndef _PROB_H_
#define _PROB_H_

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
#include <memory>
#include <algorithm>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unordered_set>


#include "utils.h"
#include "grid.h"

#define PI 3.14159265

using namespace Eigen;
using namespace std;

/*****
 * Hollinger Paper b(t+1) = b(t) * Pdt(D(N)) N-> number of agents
 */

class Prob {

public:

	Util util;
	typedef Util::Position Point;
	typedef Util::PositionInt PointInt;
	typedef Util::Hash Hash;
	

	Grid grid;

	int NUMBEROFAGENTS = 1;

	 

	struct SAgentInfo {
		Util::Position agentPosition;
		Util::Position newPosition;
		Util::Position goalPosition;
		int goalCell;
		int cellSize = 15876; // x = 25 y  = 25
		float theta;
 		// key- goal number value - position
		vector<Util::Position> goalLocationList;

		enum EAgentState {
			GoalKnown = 0, GoalUnknown = 1, GoalReached = 2
		} AgentState;


		MatrixXd d_mat; // (Numberofgridsx*Numberofgridsy,Numberofgridsx*Numberofgridsy)
		MatrixXd b_vector_new; // size (1,Numberofgridsx*Numberofgridsy)
		MatrixXd b_vector_old;

		map<int,vector<int> > parentChildCells;
		int lastknownCell;
		
		vector<int> sortedAgentLocations; // locations sorted based on the probability value
		Util::Position centroid;

		SAgentInfo();
	};

	struct SFinalAgentData{
		map <int, vector<int> > clusterMap; // key -> Agent Id value -> all the agent ids which share location

		map < vector<int> , vector<int> > clusterLocation; // key -> Agent ids which share same location
												//value -> cell Number they share
		int clusterSize;

		MatrixXd b_vector_final;
	};



public:

	int timeStep;
	//struct SAgentInfo Agent1;

	using map_dataType = std::map<uint8_t,std::shared_ptr<SAgentInfo> >;

	std::shared_ptr< std::map<uint8_t,std::shared_ptr<SAgentInfo> > > agentMap = std::make_shared<map_dataType>();

	std::shared_ptr <SFinalAgentData> clusterData = std::make_shared<SFinalAgentData>();

    ofstream dataFile;
	string fileName;

	ofstream dataBelief;
	string fileName2;

	Prob();


	virtual ~Prob() {
	}

	vector<float> getRoundedCoordinates(Point x);
	
	Point getCoordinates(int cellNumber);

	void initialise(int id, Util::Position currentLocation, vector<Util::Position> &GoalLocation);

	//std::shared_ptr< std::map<uint8_t,std::shared_ptr<Prob::SAgentInfo> > > update(int i);

	Util::Position update(int i);
    
    void GoalKnown(int id);
    void GoalUnKnown(int id);

    std::shared_ptr<Prob::SFinalAgentData> calculateFinalBeliefVec();

    vector<int> mapIndexToCellNumber(vector<int> &vectorLoc);

    void updateAgentPosition(int id, Util::Position currentLocation, vector<Util::Position> &GoalLocationList);
    void updateAgentPosition(int id, Util::Position goalLocation);
    /*****
     *  group the locations of agents and sort in descending order.
     *  a. This is done to distinguish different agents - utilise to allocate different agents to relays.
     *  b. use sorted locations to plan which location to start for relays for each agent
     *
     */
    void sortingLocations(int id);
    bool findIntersectingValues();

    Point findCentroid(vector<int>& cellNumbers);

    inline float checkCoordinatesInRange(float o)
	{
		o = (o < grid.minimum.x) ? grid.minimum.x : o;
		o = (o > grid.maximum.x) ? grid.maximum.x : o;

		return o;
	}

	void updateBeliefVector(float comm_Range,Util::Position agentNotFound);
	int getAgentState(int id);

	Util::Position getProbablePos(int id);
	int timeCount = 0;

	private:
	void resetValues(vector<int> cellNumbersToConsider);
};

#endif