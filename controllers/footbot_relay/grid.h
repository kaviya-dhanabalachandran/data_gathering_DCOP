#ifndef _GRID_H_
#define _GRID_H_

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
#include <algorithm>

#include "utils.h"

using namespace std;

class Grid{

public:

	Util util;
	typedef Util::Position Point;
	Point minimum;
	Point maximum; // maximum environment size
	int numberOfgridsX;
	int numberOfgridsY;
	int GridSize;
	map<int, Point> gridToPositionMap;
	map<vector<float>, int> reverseMap;
	map<vector<int>,int> rowColToCell;
	map<int, vector<int> > reverseArray;
	map<int,vector<int> > neighbourCells;

public:
	Grid(float maxX, float maxY);

	virtual ~Grid() {
		}

	void discretize();
	vector<int> findNeighbours(vector<int>& n);
	int getCellNumber(vector<float>& n);
};

#endif
