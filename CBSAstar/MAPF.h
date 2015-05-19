#pragma once
#include "FileReader.h"
#include "ConstraintTree.h"

/*
	Class MAPF
	Description: This class is in charge of all the pathfinding done by the agents. MAPF stands for Multiple Agent Path Finding.
	Author: Carlos Tirado
*/



class MAPF{
public:
	MAPF(string filename); // inputs information from a map (for static maps)
	MAPF(int size_x, int size_y); //inputs for random generation of map (for dynamic maps)
	~MAPF(void); //Destructor
	
	void Start(); //start the pathfinding
	void MoveEntities(); //Self explanatory
	

private:
	void StartCBSPathFinding();
	void StartSilversPathFinding();
	//TODO: Keep adding more elements throught the development of this project.
	FileReader* fr;
	Map* map;
	ConstraintTree* tree;
	vector<Agent> players;

	CBTNode* root; // The root node of the Tree

	bool broken;

	int time;
};