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
		//if (i % 2 == 1) D = D / 2;
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
	cout << "Number of players: " << players.size() << endl;
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

	//Now that the narrow path conflicts where found, lets solve them
	solveConflicts();

	// Now that several routes have been changed, lets check for another conflict type, bottleneck
	//BottleNeck();

	// We solve again the conflics (if there where any)
	//solveConflicts();

	
}

void MAPF::NarrowPath(){
	//TODO: Add size resrictions
	int toCompare = 0; // This represent the index of the element we are comparing
	for (unsigned int index = 1; index < paths.size(); index++){ // this will traverse the second compared element
		for (unsigned int i = 0; i < paths[toCompare].size() - 2; i++){ // this represents the element on the first agent
			for (unsigned int j = 0; j < paths[index].size() - 2; j++){ // this represents the element on the second agent
				int progressiveAccumulator = 0; // To calculate how many progressive steps we can find
				if (paths[toCompare][i] == paths[index][j]){ // if the elements are equal, lets see if the progress of the route is the same
					/*
						If the next element of toCompare, is the element before of the current element
					*/
					if (j >= 2){

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
								//c.m.setData(*map->getData()); 
								agent_conflicts.push_back(c); // Add it to the conflicts that need to be solved
							}
						}
					}
				}
			}
		}
	}
}

void MAPF::solveConflicts(){
	

	// First traverse the list of conflicts
	for (unsigned int i = 0; i < agent_conflicts.size(); i++){
		cout << "Conflicts found! Solving them!" << endl;
		//Now lets solve each conflict 1 by 1
		Conflicted c = agent_conflicts[i];

		switch (c.type){
		case NARROW_PATH:
			SolveNarrowPath(c);
			break;
		default:
			DefaultHelper(c);
			break;
		}


	}
	
}

void MAPF::SolveNarrowPath(Conflicted c){
	int index = getIndexOfAgent(c.agents[0]);
	vector<Node> adjacents = players[index].getTimedAdjacents()
}

void MAPF::DefaultHelper(Conflicted c){
	//Create the constraint tree
	tree = new ConstraintTree();

	//Create the root node
	root = new CBTNode();
	// Create the elements of the tree and assign them the necesary information

	// Add the agents and pre-calculated paths
	for (unsigned int j = 0; j < c.agents.size(); j++){
		int index = getIndexOfAgent(c.agents[j]);
		root->addAgent(&players[index]);
		root->AddPath(players[index].getPath());
	}



	// Add the constraints from the reservation table to the root node
	vector<Constraint> cons = map->GetReservationTableConstraints();
	for (unsigned int j = 0; j < cons.size(); j++){
		root->addConstraint(cons[j]);
	}

	//Set the cost of the node
	root->calculateCost();

	//Insert the root to the tree
	tree->insertRoot(root);

	// Now let's find a solution to the routes
	bool solutionFound = false;

	//Get the best node of the tree
	CBTNode *P = tree->getSolution();

	//While we can't find the solution
	while (!solutionFound){
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

		P = tree->getSolution(); // Set the best node in the tree to be P
	}
	// Finished solving conflicts, empty the list
	agent_conflicts.clear();
}

void MAPF::BottleNeck(){
	// 1 for loop to get the elements to compare
	for (unsigned int toCompare = 0; toCompare < players.size(); toCompare++){
		vector<int> agents_toCompare;
		vector<int> time_span;
		// for loop to get the elements in the path of the aget toCompare
		for (unsigned int index = 0; index < players[toCompare].getPath().size(); index++){
			// for loop to get the agent that is being compared
			for (unsigned int i = toCompare + 1; i < players.size(); i++){
				//for loop for the element of the path being compared
				for (unsigned int j = 0; j < players[i].getPath().size(); j++){
					// Look which elements have the same path time
					if (players[toCompare].getPath()[index] == players[i].getPath()[j]){
						if (!existsInList(agents_toCompare, i)) {
							agents_toCompare.push_back(i);
							time_span.push_back(j);
						}
					}
				}
			}
			// Now, we have all the elements that go to the same node as the one located on index
			//If there are more than 3 elements going through that node ....
			if (agents_toCompare.size() > 3){
				vector<int> toSort = time_span;
				std::sort(toSort.begin(), toSort.end()); // sort the elements
				vector<int> result;
				int index = 0;
				// See which elements are on the bottleneck (s)
				for (unsigned int i = 1; i < toSort.size(); i++){
					int difference = abs(toSort[index] - toSort[i]);
					if (difference >= 0 && difference <= 2){
						result.push_back(toSort[index]);
						result.push_back(toSort[i]);
					}

					index++;
				}
				Conflicted ce;
				ce.type = BOTTLENECK;

				/*
					See the fix done at the utils .h file. For now, no map adding.
				*/
				//Creating the submap
				//vector<int> result_id;
				//vector<int> result_time;
				for (unsigned int i = 0; i < result.size(); i++){
					// Look for element i in the time_span vector
					for (unsigned int j = 0; j < time_span.size(); j++){
						// When the element is found
						if (result[i] == time_span[j]){
							//result_id.push_back(agents_toCompare[j]);
							ce.agents.push_back(agents_toCompare[j]);
							//result_time.push_back(agents_toCompare[j]);
							break;
						}
					}
				}

				// Now we have the participants on the constraint
				/*Location lowerLocation = players[result_id[0]].getPath()[result_time[0] - 1].getLocation();
				Location upperLocation = players[result_id[0]].getPath()[result_time[0] - 1].getLocation();
				for (unsigned int i = 1; i < result_id.size(); i++){
					Location l = players[result_id[i]].getPath()[result_time[i] - 1].getLocation();
					if (l.x < lowerLocation.x){
						lowerLocation.x = l.x;
					}
					else if (l.x > upperLocation.x){
						upperLocation.x = l.x;
					}

					if (l.y < lowerLocation.y){
						lowerLocation.y = l.y;
					}
					else if (l.y > upperLocation.y){
						upperLocation.y = l.y;
					}
				}*/

				// Create the submap in base of the locations given
				//ce.m.setData(map->getSubData(lowerLocation.x, lowerLocation.y, upperLocation.x, upperLocation.y));

				// Finally, add the conclicted class to the list
				agent_conflicts.push_back(ce);
			}

		}
	}
}

void MAPF::Blocking(){

}

bool MAPF::existsInList(vector<int> list, int val){
	if (std::find(list.begin(), list.end(), val) != list.end()) return true;

	return false;
}

//Returns -1 if not found
int MAPF::getIndexOfAgent(int id){
	int result = -1;
	for (unsigned int i = 0; i < players.size(); i++){
		if (players[i].getId() == id){
			result = i;
			break;
		}
	}

	return result;
}