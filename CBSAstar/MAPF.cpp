#include "MAPF.h"

//Constructor that loads data from the file
MAPF::MAPF(string filename){
	//Loading data from the file
	fr = new FileReader(filename, true);

	//put the file 
	map = new Map(fr->data);
	
	broken = false;
	
	//Create the players using the data from the file
	for (unsigned int i = 0; i < fr->startings.size(); i++){
		int D = map->CalculateD();
		if (i % 2 == 1) D = D / 2;
		Location l = fr->startings[i];
		Location d = fr->endings[i];

		//Check for valid locations
		if (map->getValueAt(l.x, l.y) == 1){
			broken = true;
			map->printData();
			break;
		}

		if (map->getValueAt(d.x, d.y) == 1){
			broken = true;
			break;
		}

		Node node_location(0, 0, l.x, l.y);
		Node destination(0, 0, d.x, d.y);
		Agent u(node_location, destination, map, l.id, D);
		players.push_back(u);
	}

	if (broken)
		cout << "The data on the file given is incorrect, please try again" << endl;
	else cout << "Finished loading data from the file" << endl;

	

	time = 0;

}

//Constructor that asks the user what are the specifications of the grid and randomly creates it
MAPF::MAPF(int size_x, int size_y){
	//TODO: DO this when the rest is finished
}


MAPF::~MAPF(void){
	delete fr;
	delete map;
	//Dont delete the nodes, thats the tree's job
	delete tree;
}

void MAPF::Start(int type){
	if (!broken){
		switch (type){
		case 1:
			StartSilversPathFinding();
			break;
		case 2:
			StartCBSPathFinding();
			break;
		}
	}
	else {
		cout << "Broken file, exiting...." << endl;
	}
}

void MAPF::StartSilversPathFinding(){
	cout << "Calculating Routes" << endl;
	for (unsigned int i = 0; i < players.size(); i++){
		players[i].setTime(time);
		players[i].executeTimeSpaceAstar();
		paths.push_back(players[i].getPath());
	}
}

void MAPF::StartCBSPathFinding(){
	//Create the constraint tree
	tree = new ConstraintTree();
	
	//Create the root node
	root = new CBTNode();

	//Add the agents to the root node
	for (unsigned int i = 0; i < players.size(); i++){
		root->addAgent(&players[i]);
	}

	//find the individual paths of the elements
	root->CalculatePaths();

	//calculate the cost of this node
	root->calculateCost();

	//insert the root into the tree
	tree->insertRoot(root);

	bool solutionFound = false;
	//While we can't find the solution
	while (!solutionFound){
		//Get the best node of the tree
		CBTNode* P = tree->getSolution();

		//Validate the paths until a conflict occurs
		P->validatePaths();

		//If it is a goal node, end this, we found the solution
		if (P->isGoal()) {
			solutionFound = true;
			P->UpdateAgentsPaths();
		}
		else {
			P->ExpandNode();
		}
	}
}

void MAPF::MoveEntities(int type){
	switch (type){
	case 1:
		MoveBySilvers();
		break;
	case 2:
		MoveByCBS();
		break;
	}
}

void MAPF::MoveBySilvers(){
	bool finished = false;
	while (!finished){
		system("cls");
		finished = players[0].finished();
		for (unsigned int i = 0; i < players.size(); i++){
			players[i].move();
			if (i > 0) finished = finished && players[i].finished();
		}

		map->printData();
		system("pause");
	}

}

void MAPF::MoveByCBS(){
	bool finished = false;

	while (!finished){
		system("cls");
		for (unsigned int i = 0; i < players.size(); i++){
			players[i].moveEntity(time);
			map->setElement(players[i].getX(), players[i].getY(), (players[i].getId() + 2));
		}
		time++;

		finished = players[0].finished();
		for (unsigned int i = 1; i < players.size(); i++){
			finished = finished && players[i].finished(); 
		}
		map->printData();
		//TODO: Add the dynamic change of path either by d steps, or because of the dynamic change of obstacles
		//TODO: Update the constraints at the node
		
		system("pause");
	}

}

//This method will check for any conflicts with the paths of the agents
void MAPF::RevisePaths(){
	cout << "Checking the path for any conflict" << endl;
	
	//First, check for the Narrow Path conflict
	NarrowPath();

}

void MAPF::NarrowPath(){
	int toCompare = 0; // This represent the index of the element we are comparing
	for (unsigned int index = 1; index < paths.size(); index++){ // this will traverse the second compared element
		// Ugh, nested for loops, simply ugh
		for (unsigned int i = 0; i < paths[toCompare].size() - 2; i++){ // this represents the element on the first agent
			bool break_loop = false;
			for (unsigned int j = 0; j < paths[index].size() - 2; i++){ // this represents the element on the second agent
				int progressiveAccumulator = 0; // To calculate how many progressive steps we can find
				if (paths[toCompare][i] == paths[index][j]){ // if the elements are equal, lets see if the progress of the route is the same
					/*
						If the next element of toCompare, is the element before of the current element
					*/
					if (paths[toCompare][i + 1] == paths[index][j - 1]){
						/* 
							Means that the second element is also in each others route, we have a possible
							bottleneck lets check the tird element.
						*/

						if (paths[toCompare][i + 2] == paths[index][j - 2]){
							// We have a narrow path conflict
							Conflicted c;
							c.type = NARROW_PATH;
							c.agents.push_back(players[toCompare].getId());
							c.agents.push_back(players[index].getId());
							
						}
					}

				}
			}
		}
	}
}