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
		players[i].executeTimeSpaceAstar(time);
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

	CBSHelper();
}

void MAPF::CBSHelper(){
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
	time++;
	bool finished = false;
	while (!finished){
		system("cls");
		finished = players[0].finished();
		for (unsigned int i = 0; i < players.size(); i++){
			players[i].move(time); // The element at i will be the element at time = i + 1
			if (i > 0) finished = finished && players[i].finished();
		}
		time++;
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
	/*
		Fix: Also remove the part where the dead lock is solved by some other element.
		Date:02/06/2015
		Why?
		Because turns out, silvers algorithm already solves the deadlock issue, so there is no point
		of redoing the stuff.
	*/
	
	Blocking();
	solveConflicts();
	
}

void MAPF::Deadlock(){
	
	for (unsigned int toCompare = 0; toCompare < paths.size(); toCompare++){ // This represent the index of the element we are comparing
		for (unsigned int index = toCompare + 1; index < paths.size(); index++){ // this will traverse the second compared element
			for (int i = 0; i < paths[toCompare].size() - 2; i++){ // this represents the element on the first agent
				for (int j = 0; j < paths[index].size() - 2; j++){ // this represents the element on the second agent
					if ((paths[toCompare][i] == paths[index][j]) && (abs(i - j) <= 2)){ // if the elements are equal, lets see if the progress of the route is the same	
						/*
							If the next element of toCompare, is the element before of the current element
							*/
						if (j >= 2){ // This is needed in order to ensure the correct index is compared

							if (paths[toCompare][i + 1] == paths[index][j - 1]){
								/*
									Means that the second element is also in each others route, we have a possible
									bottleneck lets check the tird element.
									*/

								if (paths[toCompare][i + 2] == paths[index][j - 2]){
									// We have a narrow path conflict
									Conflicted c;
									c.type = DEADLOCK;
									c.agents.push_back(players[toCompare].getId());
									c.locations.push_back(paths[toCompare][i].getLocation());
									c.agents.push_back(players[index].getId());
									c.locations.push_back(paths[index][j].getLocation());
									c.times.push_back(i - 2);
									c.times.push_back(j + 2);
									agent_conflicts.push_back(c); // Add it to the conflicts that need to be solved
								}
							}
						}
					}
				}
			}
		}
	}
}

/*
	This method counts the elements on the critical zone of a conflict, and if a vector is added to
	it's parameters, the method will add those elements to the vector.
*/
int MAPF::countCriticalZone(Conflicted c, vector<Node>* criticalZoneNodes){
	// Get the indexes of the agents involved on the conflict
	int index0 = getIndexOfAgent(c.agents[0]);
	int index1 = getIndexOfAgent(c.agents[1]);
	
	//Get the 2 paths of the agents involved
	vector<Node> path0 = paths[index0];
	vector<Node> path1 = paths[index1];
	int result = 0;
	

	// Traverse through the two paths of the elements
	for (unsigned int i = 0; i < path0.size(); i++){
		for (unsigned int j = 0; j < path1.size(); j++){
			int difference = 0;
			bool equal = (path0[i] == path1[j]);
			if (equal) result++; // If the first element is the same, we have our first node on the list
			while (equal){
				// If there is a vector in the parameters, add to it the elements in the critical zone
				if (criticalZoneNodes) {
					if (!NodeExistsOnList(*criticalZoneNodes, path0[i + difference])){
						criticalZoneNodes->push_back(path0[i + difference]);
					}

					if (!NodeExistsOnList(*criticalZoneNodes, path0[i - difference])){
						criticalZoneNodes->push_back(path0[i - difference]);
					}
				}
				difference++;
				if (j >= difference && i >= difference) 
					equal = (path0[i + difference] == path1[j - difference]) && (path0[i - difference] == path1[j + difference]);
				else equal = false;
			}
		}
	}

	return result;
	
}

void MAPF::solveConflicts(){
	// First traverse the list of conflicts
	for (unsigned int i = 0; i < agent_conflicts.size(); i++){
		cout << "Conflicts found! Solving them!" << endl;
		//Now lets solve each conflict 1 by 1
		Conflicted c = agent_conflicts[i];

		switch (c.type){
		case DEADLOCK:
			SolveDeadLock(c);
			break;
		case BLOCKING_COMPLEX:
			SolveBlockingComplex(c);
			break;
		case BLOCKING_SIMPLE:
			SolveBlockingSimple(c);
			break;
		default:
			DefaultHelper(c);
			break;
		}


	}
	
}

void MAPF::SolveDeadLock(Conflicted c){
	// Select an element to modify on the conflicted list
	int index = getIndexOfAgent(c.agents[0]);
	int otherIndex = getIndexOfAgent(c.agents[1]);
	vector<Node> adjacents = players[index].getAdjacents2(c.locations[0], c.times[0]);
	if (adjacents.size() > 2){ // If the amount of adjacents are bigger than 2, we are on a head to head collision
		// First, order the adjacents
		std::sort(adjacents.begin(), adjacents.end());
		
		// Now, select the cheaper element that is NOT part of the route
		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (!players[otherIndex].isOnMyRoute(adjacents[i]) && !players[index].isOnMyRoute(adjacents[i])){
				// If that node is not part of the route of the other element
				players[index].AddNodeToPathAtTimeT(adjacents[i], c.times[0]);
				// Reroute the path startng from the new position
				players[index].ReroutePathUsingSpatialAstar(c.times[0]);
				break;
			}	
		}
	}
	else if (adjacents.size() <= 2){ // The elements are bigger than 3, this must be a NARROW PATH
		// Modify the elements of the players map so a new path could be found
		players[index].modifyMap(players[c.agents[1]].getPath());

		//See if you are in the other players route
		if (players[c.agents[1]].isOnMyRoute(players[index].getActualLocation())){
			//Now, look for the closest element that is not on the route, and move there
			players[index].MoveToClosestEscapeElement(false, players[index].getActualLocation());
		}
		else{
			//Otherwise, the first element of the route is your actual Location
			players[index].AddNodeToPathAtTimeT(players[index].getActualLocation(), 0);
		}

		

		//Now we reroute from the actual element to the destination using CBS
		//First we create the root and the tree
		//Create the constraint tree
		tree = new ConstraintTree();

		//Create the root node
		root = new CBTNode();
		// Create the elements of the tree and assign them the necesary information
		// Add the agents and pre-calculated paths
		int id = root->addAgent(&players[index]); // For later use
		root->AddPath(players[index].getPath());
		root->addAgent(&players[c.agents[1]]);
		root->AddPath(players[c.agents[1]].getPath());

		//Now we can actually recalculate the path
		int replanIndex = root->ReplanAgentFromLastIndex(id); // and store the index when the replanning started

		/*
			The critical zone will be called to that zone, where both the agents need to use, in other words,
			the zone in the map where the deadlock occurs.
		*/
		int criticalZoneLength = countCriticalZone(c); // get the critical path length
		// The int obtained above will help us know how many steps to wait until the element is done advancing through the critical path

		//Now we make the path wait that amount of steps
		root->WaitAtIndex(id, replanIndex, criticalZoneLength);

		//Finally, follow the process of CBS to validate the rest of the path
		tree->insertRoot(root);

		//Now, execute CBS and finish!
		CBSHelper();

	}
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


	CBSHelper();
	//Comented for now, but I believe this does the same as the method above
	//// Now let's find a solution to the routes
	//bool solutionFound = false;

	////Get the best node of the tree
	//CBTNode *P = tree->getSolution();

	////While we can't find the solution
	//while (!solutionFound){
	//	//Validate the paths until a conflict occurs
	//	P->validatePaths();

	//	//If it is a goal node, end this, we found the solution
	//	if (P->isGoal()) {
	//		solutionFound = true;
	//		P->UpdateAgentsPaths();
	//	}
	//	else {
	//		P->ExpandNode();
	//	}

	//	P = tree->getSolution(); // Set the best node in the tree to be P
	//}
	// Finished solving conflicts, empty the list
	agent_conflicts.clear();
}


void MAPF::Blocking(){
	for (unsigned int i = 0; i < players.size(); i++){ // This is to traverse through the players
		Node destination = players[i].getDestination();
		for (unsigned int j = 0; j < paths.size(); j++){ // This is to traverse the paths
			if (i != j){
				if (NodeExistsOnList(paths[j], destination)){
					// If the element is on the list, check if player i will arrive to its destination before player j
					int timeOcurrance = GetIndexAtArray(paths[j], destination);
					if (timeOcurrance >= paths[i].size()){
						/*
							If the player j needs to acces players i destination point after player i is finished,
							then we have a conflict. Now we need to check if it is a simple conflict or a complex conflict.
							*/
						Conflicted c;
						//The first player to be added is the player that needs to move
						c.agents.push_back(players[i].getId());
						c.times.push_back(timeOcurrance + 1); /* We add 1 because a node at index i, ocurrs at time i + 1*/

						if (map->adjacentHelper(destination).size() > 2){
							// If there are more than 2 adjacents to this node, we have a simple blocking state
							c.type = BLOCKING_SIMPLE;
						}
						else{
							// Else, we have a narrowpath blocking
							c.type = BLOCKING_COMPLEX;
						}

						agent_conflicts.push_back(c); // Add it to the conflicts that need to be solved
					}
				}
			}
		}

	}
}

void MAPF::SolveBlockingSimple(Conflicted c){
	int index = getIndexOfAgent(c.agents[0]); // Get the index of the agent we are going to use
	if (paths[index].size() < c.times[0]){ // If at the time stated, the element is just waiting there
		// Fill the path with the destination until the time t is reached
		int limit = c.times[0] - paths[index].size() - 1;
		for (int i = 0; i < limit; i++){
			players[index].PushElementAtTheBackOfRoute(players[index].getDestination());
		}
	}

	// Now, get an escape route and update the path with that
	players[index].MoveToClosestEscapeElement(true, paths[index][paths[index].size() -1]);

	//Now that we've got an escape route, go back to my own destination
	players[index].executeTimeSpaceAstarFromLastIndex();

	//Update the paths
	paths[index] = players[index].getPath();

}

void MAPF::SolveBlockingComplex(Conflicted c){

}

bool MAPF::existsInList(vector<int> list, int val){
	if (std::find(list.begin(), list.end(), val) != list.end()) return true;

	return false;
}

bool MAPF::NodeExistsOnList(vector<Node> list, Node val){
	if (std::find(list.begin(), list.end(), val) != list.end()) return true;

	return false;
}

int MAPF::GetIndexAtArray(vector<Node> list, Node val){
	int result = -1;
	for (int i = 0; i < list.size(); i++){
		if (val == list[i]){
			result = i;
			break;
		}
	}

	return result;
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