#include "probabilityCalculation.h"

// update the grid value in Prob::calculateFinalBeliefVec method if you change the grid value

Prob::SAgentInfo::SAgentInfo() {

	AgentState = SAgentInfo::GoalKnown;
	d_mat.resize(cellSize, cellSize);
	b_vector_old.resize(1, cellSize);
	b_vector_new.resize(1, cellSize);
	goalPosition.x = 0.0;
	goalPosition.y = 0.0;
}

Prob::Prob():grid(25.0,25.0){
	
	grid.discretize();

	clusterData->b_vector_final.resize(grid.numberOfgridsX, grid.numberOfgridsY);

	fileName2 = "beliefVector.csv";
	dataBelief.open(fileName, ios::out | ios::ate);

	fileName = "calculatedPosition.csv";
	dataFile.open(fileName, ios::out | ios::ate);
}


vector<float> Prob::getRoundedCoordinates(Point curr) {
	vector<float> f { curr.x, curr.y };
	for (int i = 0; i < f.size(); i++) {
		float interval = util.speed * util.frequencyOfUpdate;
		float m = round(f[i]/ interval);
		f[i] = m * interval;
		f[i] = checkCoordinatesInRange(f[i]);
	}
	//printf( "rounded coordnates %f -> %f,   %f -> %f \n",curr.x, f[0],curr.y,f[1]);
	return f;

}

void Prob::initialise(int id, Util::Position currentLocation, vector<Util::Position> &GoalLocationList) {

	std::shared_ptr<SAgentInfo> agentProbTemp(new SAgentInfo());

	agentProbTemp->agentPosition = currentLocation;
	agentProbTemp->goalLocationList = GoalLocationList;

	agentMap->emplace(id,agentProbTemp);

	agentMap->operator[](id)->goalPosition = agentMap->operator[](id)->goalLocationList[0];
	//printf("goal Locations b_initially %f %f \n",agentMap->operator[](id)->goalPosition.x, agentMap->operator[](id)->goalPosition.y);

	updateAgentPosition(id,agentMap->operator[](id)->goalPosition);
	
}

void printVector(vector<int> dumm) {
	for (int i = 0; i < dumm.size(); i++) {
		////cout << dumm[i] << " ";

	}
	////cout << endl;
}

bool checkCommon(std::vector<int> const& inVectorA,
		std::vector<int> const& nVectorB) {
	return std::find_first_of(inVectorA.begin(), inVectorA.end(),
			nVectorB.begin(), nVectorB.end()) != inVectorA.end();
}

vector<size_t> sort_indexes(const vector<double> &v) {

	// initialize original index locations
	vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),
			[&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

	return idx;
}

vector<int> Prob::mapIndexToCellNumber(vector<int> &vectorLoc) {
	vector<int> cellNumberVector;
	
	for (int i = 0; i < vectorLoc.size(); i++) 
	{
		if (i % 2 != 0) {
			//vector<int> v { vectorLoc[i], vectorLoc[i + 1] };
			cellNumberVector.push_back(vectorLoc[i]);
		}
	}
	
	return cellNumberVector;
}

Util::Position 
Prob::findCentroid(vector<int>& cellNumbers)
{	
	Util::Position centroid;
	vector<float> xPos(cellNumbers.size());
	vector<float> yPos(cellNumbers.size());

	//printf("xPos size %d, ypos size %d \n",xPos.size(), yPos.size());

	for(int i=0 ;i < cellNumbers.size() ; i++)
	{
		xPos[i] = grid.gridToPositionMap[cellNumbers[i]].x;
		yPos[i] = grid.gridToPositionMap[cellNumbers[i]].y;
	}

	centroid.x = accumulate(xPos.begin(), xPos.end(), 0.0)/xPos.size();
	centroid.y = accumulate(yPos.begin(), yPos.end(), 0.0)/yPos.size();

	//printf("centroiddd %f  %f \n", centroid.x, centroid.y);

	return centroid;
} 

void Prob::sortingLocations(int id) {
	/****
	 *  a. Convert belief vec to sparse matrix
	 *  b. Get the non-zero indices and values
	 *  c. Arg sort values -> gives the sorted matrix
	 *	d. Find the centroid of the locations to set relay goal
	 */
	vector<double> probValues;
	MatrixXi probIndices;
	int ind = 0;
	//printf("sorting \n");

	SparseMatrix<double> sparseBeliefVector(grid.numberOfgridsX,
			grid.numberOfgridsY);
	sparseBeliefVector = (agentMap->operator[](id)->b_vector_old).sparseView();
	////printf("Sparse vector size %d\n", (int)sparseBeliefVector.nonZeros());
	probIndices.resize(sparseBeliefVector.nonZeros(), 2);
	/*for(int i =0 ; i < probIndices.rows(); i++)
	{
		////printf("dummy %d %d\n", probIndices(i,0), probIndices(i,1));
	}*/
	////printf("Convert belief vec to sparse matrix \n");

	for (int k = 0; k < sparseBeliefVector.outerSize(); ++k) {
		for (SparseMatrix<double>::InnerIterator it(sparseBeliefVector, k); it;
				++it) {
			////printf("value %f , row  %f ,col %f \n", it.value(), it.row(), it.col());
			probValues.push_back(it.value());
			probIndices.row(ind) << it.row(), it.col();
			ind = ind + 1;
		}
	}

	////printf("Get the non-zero indices and values \n");

	int in = 0;
	MatrixXi tempMat(probIndices.rows(), probIndices.cols());
	////cout << "probIndices " << probIndices.rows() << "," << probIndices.cols()<< endl;
	////cout << probIndices << endl;
	for (auto i : sort_indexes(probValues)) {
		////printf("sorting indexes %d \n", i);

		tempMat.row(in) = probIndices.row(i);
		in = in + 1;
	}

	////printf("Arg sort values -> gives the sorted matrix \n");

	// Eigen Matrix to vector of locations
	tempMat.transposeInPlace();
	
	vector<int> matToVector(tempMat.rows() * tempMat.cols());
	Eigen::Map<Eigen::MatrixXi>(matToVector.data(), tempMat.rows(),
			tempMat.cols()) = tempMat;

	
	// vector of locations -> vector of cell numbers in grid of the locations
	agentMap->operator[](id)->sortedAgentLocations = mapIndexToCellNumber(matToVector);
	agentMap->operator[](id)->centroid = findCentroid(agentMap->operator[](id)->sortedAgentLocations);

	////printf("Find the centroid of the locations to set relay goal\n");
	////printf("%d Location size %f %f \n",(id , agentMap->operator[](id)->sortedAgentLocations.size()));

	Point largestValueLoc = getCoordinates(agentMap->operator[](id)->sortedAgentLocations[0]);
	//dataFile << id << "," << agentMap->operator[](id)->centroid.x << "," << agentMap->operator[](id)->centroid.y << "," << largestValueLoc.x << "," << largestValueLoc.y << "\n";
}

void Prob::GoalKnown(int id) {

	//printf("theta %f \n", agentMap->operator[](id)->theta);
	//new Position
	//printf( "id and Position %d %f %f\n", id, agentMap->operator[](id)->agentPosition.x, agentMap->operator[](id)->agentPosition.y);
	/*agentMap->operator[](id)->newPosition.x = agentMap->operator[](id)->agentPosition.x
			+ (util.frequencyOfUpdate * util.speed * cos(agentMap->operator[](id)->theta));
	agentMap->operator[](id)->newPosition.y = agentMap->operator[](id)->agentPosition.y
			+ (util.frequencyOfUpdate * util.speed * sin(agentMap->operator[](id)->theta));*/

	agentMap->operator[](id)->newPosition.x = agentMap->operator[](id)->agentPosition.x
			+ (1 * util.speed * cos(agentMap->operator[](id)->theta));
	agentMap->operator[](id)->newPosition.y = agentMap->operator[](id)->agentPosition.y
			+ (1 * util.speed * sin(agentMap->operator[](id)->theta));

	//printf("Position %f %f\n",agentMap->operator[](id)->newPosition.x, agentMap->operator[](id)->newPosition.y);

	if(agentMap->operator[](id)->newPosition.x >= 10 || agentMap->operator[](id)->newPosition.x >= 10)
	{
		//printf("current goal Position %f %f\n",agentMap->operator[](id)->goalPosition.x, agentMap->operator[](id)->goalPosition.y);
	}

	//new CellNumber
	/*vector<float> cellCoordinates = getRoundedCoordinates(
			agentMap->operator[](id)->newPosition);
	int cellNumber = grid.getCellNumber(cellCoordinates);
	//printf("new_coo %f %f %d\n", cellCoordinates[0],cellCoordinates[1],cellNumber);

	// Assigning Probabilities
	agentMap->operator[](id)->b_vector_old.fill(0.0);
	

	agentMap->operator[](id)->b_vector_old(cellNumber) = 0.5;
	vector<int> neighbors = grid.neighbourCells[cellNumber];
	for (int i = 0; i < neighbors.size(); i++)
		agentMap->operator[](id)->b_vector_old(neighbors[i]) = 0.5 / neighbors.size();*/

	
	agentMap->operator[](id)->agentPosition = agentMap->operator[](id)->newPosition;

	if (util.findDistance(&agentMap->operator[](id)->agentPosition,
			&agentMap->operator[](id)->goalPosition) <= 0.4) {
		agentMap->operator[](id)->AgentState = SAgentInfo::GoalReached;
		//printf("GOAL REACHED \n");
		agentMap->operator[](id)->b_vector_old.fill(0.0);

		//agentMap->operator[](id)->lastknownCell = cellNumber;
		//agentMap->operator[](id)->b_vector_old(agentMap->operator[](id)->lastknownCell) = 1.0;
		//agentMap->operator[](id)->d_mat(agentMap->operator[](id)->lastknownCell,
		//		agentMap->operator[](id)->lastknownCell) = 1.0;
	}

	//sortingLocations(id)
	agentMap->operator[](id)->centroid = agentMap->operator[](id)->newPosition;

	dataFile << id << "," << agentMap->operator[](id)->newPosition.x << "," << agentMap->operator[](id)->newPosition.y << "\n";

	//printf("IN PROB current Position %f %f\n",agentMap->operator[](id)->newPosition.x, agentMap->operator[](id)->newPosition.y);
	//printf("IN PROB goal Position %f %f\n",agentMap->operator[](id)->goalPosition.x, agentMap->operator[](id)->goalPosition.y);

}

void Prob::GoalUnKnown(int id) {
	////cout << "GOAL UNKNOWN" << endl;

	////printf << agentMap[id]->d_mat << endl;
	//////cout << agentMap[id]->b_vector_old << endl;

	map<int, vector<int> > tempCells = agentMap->operator[](id)->parentChildCells;
	agentMap->operator[](id)->parentChildCells.clear();

	/*
	 * Loop 1 -> map with parent and children
	 * Loop 2 -> iterating children to find their neighbours
	 * Loop 3 -> iterating neighbours to set their values in D matrix
	 */
	for (map<int, vector<int> >::iterator it = tempCells.begin();
			it != tempCells.end(); ++it) {

		vector<int> oldChildren = it->second;
		std::sort(oldChildren.begin(), oldChildren.end());
		////printf("old children \n");
		//printVector(oldChildren);

		for (int c = 0; c < oldChildren.size(); c++) {

			vector<int> neighbors = grid.neighbourCells[oldChildren[c]];
			std::sort(neighbors.begin(), neighbors.end());
			//neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), it->first), neighbors.end());
			////printf("neighbours \n");
			//printVector(neighbors);

			if (checkCommon(oldChildren, neighbors)) {
				std::vector<int> newNeighbors;
				std::set_difference(neighbors.begin(), neighbors.end(),
						oldChildren.begin(), oldChildren.end(),
						std::back_inserter(newNeighbors));
				neighbors = newNeighbors;
			}

			////printf("new Neighbours \n");
			//printVector(neighbors);

			agentMap->operator[](id)->parentChildCells.emplace(oldChildren[c], neighbors);
			
			for (auto const &ele:agentMap->operator[](id)->parentChildCells) 
			{
        		////printf(" Parent %d", ele.first);  
        		vector<int> children =  ele.second;
        		/*for(auto const &e:children)
        		{
        			//printf("children %d \n", e);
        		}*/
    		}

			for (int n = 0; n < neighbors.size(); n++) {
				////printf("iterating neighbors loop \n");
				//agentMap->operator[](id)->d_mat(oldChildren[c], neighbors[n]) = agentMap[id]->d_mat(oldChildren[c], neighbors[n]) + agentMap[id]->d_mat(it->first, oldChildren[c])/ neighbors.size();
				double value = 1.0 / (neighbors.size() * 1.0);
				////printf("value to dmat %f\n", value);
				agentMap->operator[](id)->d_mat(oldChildren[c], neighbors[n]) = value;
				//////cout << "value " << value << endl;
			}
			// make  value of the parent as 0.0
			agentMap->operator[](id)->d_mat(it->first, oldChildren[c]) = 0.0;
			//////cout << oldChildren[c] << endl;
			//////cout << "d_"
			//////cout << agentMap[id]->d_mat << endl;
		}
	}
   
	// finding belief vector and assigning new to old
	agentMap->operator[](id)->b_vector_new = agentMap->operator[](id)->b_vector_old * agentMap->operator[](id)->d_mat;
	agentMap->operator[](id)->b_vector_old = agentMap->operator[](id)->b_vector_new;

	

	////printf("b_vec row size , col size %d %d  \n", int(agentMap->operator[](id)->b_vector_old.rows()), int());
	/*for (int i = 0; i < grid.numberOfgridsX * grid.numberOfgridsY; i++) {
		if (i > 0 && i % grid.numberOfgridsX == 0)
			//printf("\n");
		////cout << agentMap->operator[](id)->b_vector_old(i) << " ";
	}
	//printf("\n");*/
	////printf("Mat sum %d\n",agentMap->operator[](id)->b_vector_old.sum());
	//printf("Mat sum %d\n",int(agentMap->operator[](id)->b_vector_old.sum()));

	sortingLocations(id);
}

void Prob::updateAgentPosition(int id, Util::Position currentLocation, vector<Util::Position> &goalLocationList)
{       
		Util::Position currentTarget = goalLocationList[0];
		if(not util.comparePosition(&currentTarget,&agentMap->operator[](id)->goalPosition))
		{
		    // initialised to new current and goal Positions
		    agentMap->operator[](id)->agentPosition = currentLocation;
		    agentMap->operator[](id)->goalPosition = goalLocationList[0];

			vector<float> cellCoordinates = getRoundedCoordinates(currentLocation);
			int f = grid.getCellNumber(cellCoordinates);
			agentMap->operator[](id)->b_vector_old.fill(0.0);
			agentMap->operator[](id)->b_vector_old(f) = 1.0;

			std::vector<float> goalP = getRoundedCoordinates(agentMap->operator[](id)->goalPosition);
			//////cout << "goal " << goalP[0] << "," << goalP[1] << endl;
			agentMap->operator[](id)->goalCell = grid.getCellNumber(goalP);

			//printf( "GCell %d \n",agentMap->operator[](id)->goalCell);

			agentMap->operator[](id)->theta = util.findTheta(&agentMap->operator[](id)->agentPosition,&agentMap->operator[](id)->goalPosition);
		}
		else
		{
			agentMap->operator[](id)->agentPosition = currentLocation;
		}
		//agentMap->operator[](id)->AgentState = SAgentInfo::GoalKnown;

		agentMap->operator[](id)->goalLocationList = goalLocationList;
		
}


void Prob::updateAgentPosition(int id, Util::Position goalLocation)
{			
			//printf("update called from prob class \n");
			// initialised to new current and goal Positions
		    
		    agentMap->operator[](id)->goalPosition = goalLocation;

			vector<float> cellCoordinates = getRoundedCoordinates(agentMap->operator[](id)->agentPosition);
			int f = grid.getCellNumber(cellCoordinates);
			agentMap->operator[](id)->b_vector_old.fill(0.0);
			agentMap->operator[](id)->b_vector_old(f) = 1.0;

			std::vector<float> goalP = getRoundedCoordinates(agentMap->operator[](id)->goalPosition);
			//////cout << "goal " << goalP[0] << "," << goalP[1] << endl;
			agentMap->operator[](id)->goalCell = grid.getCellNumber(goalP);

			//printf( "GCell %d \n",agentMap->operator[](id)->goalCell);

			agentMap->operator[](id)->theta = util.findTheta(&agentMap->operator[](id)->agentPosition,&agentMap->operator[](id)->goalPosition);
}

//std::shared_ptr< std::map<uint8_t,std::shared_ptr<Prob::SAgentInfo> > > 
Util::Position Prob::update(int id) {
	//std::shared_ptr<SAgentInfo> agentData = agentMap->operator[](id);
	switch (agentMap->operator[](id)->AgentState) {
	case SAgentInfo::GoalKnown: {
		//printf("Goal Known case \n");
		GoalKnown(id);
		break;
	}
	case SAgentInfo::GoalReached: 
	{
		//printf("Goal Reached \n");
		
		if(timeCount == 7)
		{   
			timeCount = 0; 
			agentMap->operator[](id)->agentPosition = agentMap->operator[](id)->goalLocationList[0];
			agentMap->operator[](id)->goalLocationList.erase(agentMap->operator[](id)->goalLocationList.begin());

			if(agentMap->operator[](id)->goalLocationList.size() > 0)
			{
				agentMap->operator[](id)->AgentState = SAgentInfo::GoalKnown;
				//printf("future Goal location known %f %f \n",agentMap->operator[](id)->goalLocationList[0].x , agentMap->operator[](id)->goalLocationList[0].y);
				updateAgentPosition(id, agentMap->operator[](id)->goalLocationList[0]);
		    	
			}
			else
			{  
				//printf("Future goal not known \n");
				vector<int> agentDetectedCell { agentMap->operator[](id)->lastknownCell };
				agentMap->operator[](id)->AgentState = SAgentInfo::GoalUnknown;
				//printf("last known cell %d \n", agentMap->operator[](id)->lastknownCell);

				agentMap->operator[](id)->b_vector_old.fill(0.0);
				agentMap->operator[](id)->b_vector_old(agentMap->operator[](id)->lastknownCell) = 1.0;
				agentMap->operator[](id)->parentChildCells.emplace(agentMap->operator[](id)->lastknownCell,agentDetectedCell);
			}
		}
		else
		{
			timeCount = timeCount + 1;
			//printf(" Goal Reached waiting \n");
		}
		break;
	}
	case SAgentInfo::GoalUnknown: {
		//printf("Goal Unknown \n");
		GoalUnKnown(id);
		break;
	}

	}

	return agentMap->operator[](id)->centroid;
}

/*bool
Prob::findIntersectingValues()
{

bool flag = false;

for (map_dataType::iterator i=agentMap->begin(); i!=agentMap->end(); ++i) 
	{   
		for(int j=i->first+1 ; j < agentMap->size(); j++)
		{
			std::sort(i->second->sortedAgentLocations.begin(),i->second->sortedAgentLocations.end());
			std::sort(agentMap->operator[](j)->sortedAgentLocations.begin(),agentMap->operator[](j)->sortedAgentLocations.end());

			vector<int> commonCells;
			vector<int> ::iterator it;
			it = std::set_intersection (i->second->sortedAgentLocations.begin(),i->second->sortedAgentLocations.end(), agentMap->operator[](j)->sortedAgentLocations.begin(),agentMap->operator[](j)->sortedAgentLocations.end(), commonCells.begin());

			commonCells.resize(it-commonCells.begin());
			if(commonCells.size() > 0)
			{
				vector<int> agentIds{i->first,j};

				clusterData->clusterLocation.emplace(agentIds ,commonCells);

				map<int, vector<int> >::iterator itr = clusterData->clusterMap.find(i->first);

					if(itr != clusterData->clusterMap.end())
					{
						clusterData->clusterMap[i->first].push_back(j);
					}
					else
					{
				  		vector<int> tempAgentId{j};
				  		clusterData->clusterMap.emplace(i->first, tempAgentId);
					}

				flag = true;
			}

		}

	}
	//printf("No problems");
	return flag;
}

std::shared_ptr<Prob::SFinalAgentData> Prob::calculateFinalBeliefVec() {
	// Probability of occurrence of anyone of the random events
	MatrixXd sum;
	MatrixXd pdt;

	sum.resize(grid.numberOfgridsX, grid.numberOfgridsY);
	pdt.resize(grid.numberOfgridsX, grid.numberOfgridsY);

	sum.fill(0.0);
	pdt.fill(0.0);
	//pdt.setIdentity(pdt.rows(), pdt.cols());
	map<int, MatrixXd> tempMap;

	for (map_dataType::iterator agent=agentMap->begin(); agent!=agentMap->end(); ++agent) 
	{
		MatrixXd temp;

		//= agentMap[agent.first]->b_vector_old;
		temp.resize(grid.numberOfgridsX, grid.numberOfgridsY);

		for (int i = 0; i < grid.numberOfgridsX; i++) {
			int c = i * grid.numberOfgridsX;
			temp.row(i)
					<< agentMap->operator[](agent->first)->b_vector_old.block<1, 51>(0, c);
		}

		sum = sum + temp;
		tempMap.emplace(agent->first, temp);

		//printf("Sum calculated \n");
	}
	if (findIntersectingValues())
	{

		for(auto const &id: clusterData->clusterLocation)
		{
			vector<int> cellNum = id.second;
			for(int i=0 ; i< cellNum.size() ; i++)
			{
				vector<int> loc = grid.reverseArray[cellNum[i]];
				int a = tempMap[id.first[0]](loc[0], loc[1]); // agent 1
				int b = tempMap[id.first[1]](loc[0], loc[1]); // agent 2

				pdt(loc[0], loc[1]) = pdt(loc[0], loc[1]) > 0 ? pdt(loc[0], loc[1]) * a * b : pdt(loc[0], loc[1]);
			}
		}
	}


	clusterData->b_vector_final = sum - pdt;
	//printf("Final done \n");
	/*
	 *  Write belief vector to a file
	 */
	/*IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n", "", "", "","\n");

	//dataBelief << agentMap->operator[](1)->b_vector_old.format(CSVFormat);
	dataBelief << clusterData->b_vector_final.format(CSVFormat);

	clusterData->b_vector_final = clusterData->b_vector_final / clusterData->b_vector_final.sum();

	/*cv::Mat_<float> a,dst;

	cv::eigen2cv(clusterData->b_vector_final,a);
	cv::normalize(a, dst, 0, 1, cv::NORM_MINMAX);
	cv::imshow("test", dst);
	cv::waitKey(0);*/

	/*//printf("Final \n");
	////cout << clusterData->b_vector_final << endl;
	//cout<< "sum final " << clusterData->b_vector_final.sum() << endl;

	/*for(auto& a: sortedAgentLocations)
	 {
	 ////cout << "id " << a.first << endl;
	 ////cout << "locations " << endl;
	 ////cout << a.second << endl;
	 }*/

	/*
	 *  Write belief vector to a file
	 */
	//IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n", "", "", "",
	//		"\n");

	//dataBelief << clusterData->b_vector_final.format(CSVFormat);
	/*return clusterData;

}*/


Util::Position Prob::getCoordinates(int cellNumber)
{
	return grid.gridToPositionMap[cellNumber];
}


void Prob::resetValues(vector<int> cellNumbersToConsider) 
{
	for (map_dataType::iterator i = agentMap->begin(); i != agentMap->end();
			++i) 
	{
		if(not agentMap->operator[](i->first)->AgentState == 0)
		{
			for (auto& pc : agentMap->operator [](i->first)->parentChildCells) 
			{
				vector<int> removeCell(20);
				vector<int> loc = pc.second;
				sort(cellNumbersToConsider.begin(), cellNumbersToConsider.end());
				sort(loc.begin(), loc.end());
				vector<int>::iterator it = set_intersection(
						cellNumbersToConsider.begin(), cellNumbersToConsider.end(),
						loc.begin(), loc.end(), removeCell.begin());
				removeCell.resize(it - removeCell.begin());
				for (auto& rem : removeCell) 
				{
					loc.erase(std::remove(loc.begin(), loc.end(), rem), loc.end());
					agentMap->operator [](i->first)->b_vector_old(rem) = 0.0;
				}
				if (removeCell.size() > 0) 
				{
					agentMap->operator [](i->first)->parentChildCells[pc.first] =
							loc;
					agentMap->operator [](i->first)->b_vector_old =
							agentMap->operator [](i->first)->b_vector_old
									/ agentMap->operator [](i->first)->b_vector_old.sum();
				}
			}
		}
	}
}

void Prob::updateBeliefVector(float comm_Range, Util::Position agentNotFound)
{   
	//printf("I am here in probabilityCalculation \n");
	int NumberOfCellsToConsider = int(comm_Range/(util.speed*util.frequencyOfUpdate));
	vector<float> agentPos{agentNotFound.x,agentNotFound.y};

	//cout << "cell Number " << grid.getCellNumber(agentPos);

	int cellNumber = grid.getCellNumber(agentPos);

	vector<int> cellNumbersToConsider;
	vector<int> temp(NumberOfCellsToConsider*2+1);
	std::iota(temp.begin(),temp.end(),-1*NumberOfCellsToConsider);
	temp.erase(temp.begin()+NumberOfCellsToConsider);
	//cout << temp.size() << "," << NumberOfCellsToConsider << endl;

	cellNumbersToConsider.push_back(cellNumber);
	for(int i=0 ; i < temp.size(); i++)
	{
		int val = cellNumber+temp[i];
		cellNumbersToConsider.push_back(val);
		//printf("val %d ", val);
		for(int j=0; j < temp.size(); j++)
		{
			cellNumbersToConsider.push_back(val+(temp[j]*grid.numberOfgridsX));
			//printf("val %d ", val+temp[j]);
		}
	}



	resetValues(cellNumbersToConsider);
}


int Prob::getAgentState(int id)
{
	return agentMap->operator[](id)->AgentState;
}

Util::Position Prob::getProbablePos(int id)
{
	return agentMap->operator[](id)->centroid;
}