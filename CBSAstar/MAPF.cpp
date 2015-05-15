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

	tree = new ConstraintTree();

}

//Constructor that asks the user what are the specifications of the grid and randomly creates it
MAPF::MAPF(int size_x, int size_y){
	//TODO: DO this when the rest is finished
}


MAPF::~MAPF(void){
	delete fr;
	delete map;
	//TODO: Dont delete the nodes, thats the tree's job
	delete tree;
}

void MAPF::Start(){
	if (!broken){
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
			if (P->isGoal()) solutionFound = true;
			else {
				P->ExpandNode();
			}
		}
		
	}
	else {
		cout << "Broken file, exiting...." << endl;
	}
}

void MAPF::MoveEntities(){
	//TODO: Left here, implement the movement of the entities
	for (unsigned int i = 0; i < players.size(); i++){
		players[i].move();
	}
}