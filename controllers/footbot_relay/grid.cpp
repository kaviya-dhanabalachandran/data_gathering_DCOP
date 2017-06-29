#include "grid.h"

Grid::Grid(float maxX, float maxY) {
	maximum.x = maxX; // 3m along x
	maximum.y = maxY; // 3m along y

	minimum.x = 0.0;
	minimum.y = 0.0;

	numberOfgridsX = ceil(maximum.x / (util.speed * util.frequencyOfUpdate))
			+ 1;
	numberOfgridsY = ceil(maximum.y / (util.speed * util.frequencyOfUpdate))
			+ 1;

	GridSize = numberOfgridsX * numberOfgridsY;
}

vector<int> Grid::findNeighbours(vector<int>& cellIndex) {
	int MIN_X = 0;
	int MAX_X = numberOfgridsX;
	int MIN_Y = 0;
	int MAX_Y = numberOfgridsY;

	/*	int startPosX =
	 (cellIndex[0] - 1 <= MIN_X) ? cellIndex[0] : cellIndex[0] - 1;
	 int startPosY =
	 (cellIndex[1] - 1 <= MIN_Y) ? cellIndex[1] : cellIndex[1] - 1;
	 int endPosX = (cellIndex[0] + 1 > MAX_X) ? cellIndex[0] : cellIndex[0] + 1;
	 int endPosY = (cellIndex[1] + 1 > MAX_Y) ? cellIndex[1] : cellIndex[1] + 1; */

	int startPosX =
			(cellIndex[0] - 1 < MIN_X) ? cellIndex[0] : cellIndex[0] - 1;
	int startPosY =
			(cellIndex[1] - 1 < MIN_Y) ? cellIndex[1] : cellIndex[1] - 1;
	int endPosX = (cellIndex[0] + 1 < MAX_X) ? cellIndex[0] + 1 : cellIndex[0];
	int endPosY = (cellIndex[1] + 1 < MAX_Y) ? cellIndex[1] + 1 : cellIndex[1];

	//cout << "start: x " << startPosX << " end " << endPosX << endl;
	//cout << "start: y " << startPosY << " end " << endPosY << endl;

	vector<int> neighbours;
	// See how many are alive
	for (int rowNum = startPosX; rowNum <= endPosX; rowNum++) {
		for (int colNum = startPosY; colNum <= endPosY; colNum++) {
			// All the neighbors will be grid[rowNum][colNum]
			if (not (rowNum == cellIndex[0] && colNum == cellIndex[1])) {
				std::vector<int> v { rowNum, colNum };
				neighbours.push_back(rowColToCell[v]);
			}
		}
	}
	return neighbours;
}

void Grid::discretize() {
	Point temp;
	int counter = 0;
	cout << numberOfgridsX << "," << numberOfgridsY << endl;
	for (int i = 0; i < numberOfgridsY; ++i) {
		temp.y = i * util.speed * util.frequencyOfUpdate;

		for (int j = 0; j < numberOfgridsX; ++j) {

			temp.x = j * util.speed * util.frequencyOfUpdate;

			//std::vector<float> v { temp.x, temp.y };

			// array -> row, col values contains cell number, map -> key (cell number), value (row,col)

			std::vector<int> v2 { j,i };
			rowColToCell.emplace(v2, counter);
			reverseArray.emplace(counter, v2);

			// key -> cell Number Value -> coordinates
			gridToPositionMap.emplace(counter, temp);
			//reverseMap.emplace(v, counter);

			counter = counter + 1;
		}

		temp.x = 0.0;
	}



	/*	for(int i =0 ;i < numberOfgridsX * numberOfgridsY; i++)
	{
		Util::Position te = gridToPositionMap[i];
		cout << i << " " << te.x << "," << te.y  << endl;

		std::vector<int> v2 = reverseArray[i];
		cout << i << " " << v2[0] << "," << v2[1] << endl;
	}

		for (std::map<vector<float>, int>::iterator it=reverseMap.begin(); it!=reverseMap.end(); ++it)
	  {

	    std::cout << it->first[0] << "," << it->first[1] << ' ' << it->second << endl;
	  }
*/

	for (int i = 0; i < numberOfgridsX * numberOfgridsY; i++) {
		vector<int> x = reverseArray[i];
		vector<int> n = findNeighbours(x);
		neighbourCells.emplace(i, n);
	}

}

int Grid::getCellNumber(vector<float>& cellCoordinates) {
    cout << "test " << cellCoordinates[0] << " " << cellCoordinates[1] << " " << typeid(cellCoordinates[0]).name() << endl;

    vector<int> tem{int(cellCoordinates[0]/(util.speed*util.frequencyOfUpdate)),int(cellCoordinates[1]/(util.speed*util.frequencyOfUpdate))};

  /*  auto search = rowColToCell.find(tem);
        if(search != rowColToCell.end()) {
            std::cout << "Found " << search->first[0] << "," << search->first[1] << " " << search->second << '\n';
        }
        else {
            std::cout << "Not found\n";
        }
*/
	int t = rowColToCell[tem];
	cout << "cell Number " << t << endl;
	return rowColToCell[tem];
}

