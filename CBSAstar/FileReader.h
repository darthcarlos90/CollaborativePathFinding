#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "Matrix.h"

using namespace std;


/*
	Struct created to easily group the locations being read.
*/
struct location{
	location(int x, int y, int id){
		this->x = x;
		this->y = y;
		this->id = id;
	};

	int x;
	int y;
	int id; //id of tthis location (id of the unit-to-be)
};




/*
	This class was created to store the information obtained from the text
	file.
	Format of the text file:
	Rows R
	Columns C
	Entities E
	x0 y0 xf0 yf0
	x1 y1 xf1 yf1
	.
	.
	.
	.
	.
	xE-1 yE-1 xfE-1 yfE-1
	xE yE xfE yfE
	Map
	Where:
		- R: The number of rows in the map
		- C: The number of columns in the map
		- E: The number of entities in the map
		- xn, yn : The starting coordinates of the entity n
		- xfn yfn : The destination coordinates of the entity n
		- Map : The map representing 0 with normal nodes, and 1 with
			obstacle nodes.
*/

class FileReader{
public:
	FileReader(std::string file, bool time_space);
	~FileReader(void);

	// I'm too lazy to create getters and setters, so I'll just set every
	// variable public. No remorse, no regret.
	bool time_space;
	int rows;
	int columns;
	int players;
	vector<location> startings;
	vector<location> endings;

	Matrix<int>* data;// I feel I'm gonna regret making this public 

private:
	void readData(std::string filename);
	
	
};