#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "Matrix.h"
#include "Utils.h"

using namespace std;

/*
	Update: This class will not only manage obtaining information from a
	text file, but also saving information to a textfile.
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

class FileManager{
public:
	FileManager(std::string file, bool time_space);
	// Empty constructor used to create file writer
	FileManager(std::string outputFile);
	~FileManager(void);

	// I'm too lazy to create getters and setters, so I'll just set every
	// variable public. No remorse, no regret.
	bool time_space;
	int rows;
	int columns;
	int players;
	vector<Location> startings;
	vector<Location> endings;
	ofstream myfile;
	Matrix<int>* data;// I feel I'm gonna regret making this public 
	
	
	void closeFile();

private:
	void readData(std::string filename);
	std::string outputFile;
	
	bool outputClosed;
	
};