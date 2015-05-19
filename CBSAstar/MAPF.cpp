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
		Location l = fr->startings[i];
		Location d = fr->endings[i];

		//Check for valid locations
		if (map->getValueAt(l.x, l.y) == 1){
			broken = true;
			break;
		}

		if (map->getValueAt(d.x, d.y) == 1){
			broken = true;
			break;
		}

		Node node_location(0, 0, l.x, l.y);
		Node destination(0, 0, d.x, d.y);
		Agent u(node_location, destination, map, l.id, 4);
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
			players[i].move(time);
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