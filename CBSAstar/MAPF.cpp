#include "MAPF.h"

//Constructor that loads data from the file
MAPF::MAPF(string filename){
	//Loading data from the file
	fr = new FileReader(filename, true);

	//put the file 
	map = new Map(fr->data);
	broken = false;
	if (map->hasData()){
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
	else {
		broken = true;
	}
	
	

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
	/*
		Fix: Now the CBS needs a parameter to know on who run the CBS, the players given by the user, or
		some other custom players.
		Date: 10/06/2015
		Why?
		Re-use of code
	*/
	
	RunCBSUsingPlayers(players);

}


void MAPF::CBSHelper(bool RunCheck){
	bool solutionFound = false;
	//While we can't find the solution
	while (!solutionFound){
		//Get the best node of the tree
		CBTNode* P = tree->getSolution();
		
		// So that certain special cases could be checked
		if (RunCheck){
			P->FindSpecialCases();
		}

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

void MAPF::RunCBSUsingPlayers(vector<Agent> agents){
	//Create the constraint tree
	tree = new ConstraintTree();

	//Create the root node
	root = new CBTNode();

	//Add the agents to the root node
	for (unsigned int i = 0; i < agents.size(); i++){
		root->addAgent(&agents[i]);
	}

	//find the individual paths of the elements
	root->CalculatePaths();

	//calculate the cost of this node
	root->calculateCost();

	//insert the root into the tree
	tree->insertRoot(root);
	
	CBSHelper(false);

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
	bool verify = false;
	while (!finished){
		system("cls");
		map->cleanMap();
		finished = players[0].finished();
		for (unsigned int i = 0; i < players.size(); i++){
			players[i].move(time); // The element at i will be the element at time = i + 1
			if (i > 0) finished = finished && players[i].finished();
			verify = verify || players[i].NeedsPathVerification(); // When An element just updated its path
		}
		// If an element just changed it's path, verify it
		if (verify){
			verify = false;
			// Update the paths on the lists
			for (unsigned int i = 0; i < players.size(); i++){
				paths[i] = players[i].getPath();
				players[i].SetPathVerificationFlag(false);
			}
			//Now revise the paths
			RevisePaths();
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
			unsigned int difference = 0;
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

#pragma region probablyunused

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


void MAPF::SolveDeadLock(Conflicted c){
	// Select an element to modify on the conflicted list
	int index = getIndexOfAgent(c.agents[0]);
	int otherIndex = getIndexOfAgent(c.agents[1]);
	Node temp(0, c.locations[0]);
	vector<Node> adjacents = players[index].getTimedAdjacentsWithoutParents(temp, c.times[0]);
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
			players[index].MoveToClosestEscapeElement(false, players[index].getLocation());
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
		CBSHelper(false);

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


	CBSHelper(false);

	agent_conflicts.clear();
}
#pragma endregion

bool MAPF::IsSubset(vector<int> a, vector<int> b){
	std::sort(a.begin(), a.end());
	std::sort(b.begin(), b.end());
	return std::includes(a.begin(), a.end(), b.begin(), b.end());
}
int MAPF::AlreadyOnConflict(vector<int> agents){
	int index = -1; // Return -1 if not found
	for (unsigned int i = 0; i < agent_conflicts.size(); i++){
		if (IsSubset(agents, agent_conflicts[i].agents)){
			index = i;
			break;
		}
	}

	return index;
}


// This method simply calls the helper methods for blocking
void MAPF::Blocking(){
	MultipleBlocking();
	SimpleBlocking();
}

// A helper function for the helper functions. Helperfunctionsception. OK bad joke.
bool MAPF::DetectBlockingHelper(unsigned int currentPlayer, unsigned int currentPath, Node destination, unsigned int* timeOc){
	bool result = false;
	if (NodeExistsOnList(paths[currentPath], destination)){
		// If the destination is on the path, check if the currentPlayer will arrive to the destination before currentPath player
		unsigned int timeOcurrance = GetIndexAtArray(paths[currentPath], destination.getLocation());
		if (timeOcurrance >= paths[currentPlayer].size()){
			/*
				If the currentPlayer needs to acces currentPath player's destination point after player i is finished,
				then we have a conflict.
			*/
			if (timeOc) *timeOc = timeOcurrance;
			result = true;
		}
	}

	return result;
}

void MAPF::MultipleBlocking(){
	for (unsigned int i = 0; i < players.size(); i++){ // This for is to see through the players
		Node destination = players[i].getDestination();
		int blockedElements = 0;
		Conflicted c;
		for (unsigned int j = 0; j < paths.size(); j++){
			if (i != j){
				if (DetectBlockingHelper(i, j, destination)){
					blockedElements++;
					c.agents.push_back(j);
				}
			}
		}
		// If this element is blocking various paths
		if (blockedElements > 1){
			c.type = BLOCKING_MULTIPLE;
			//TODO: Left here
			/*
				After a multiple block has been identified, now I need to check if this multiple block is not part of a bigger multiple block.
				If it is already part of a multiple block, dont add it to the conflicts, otherwise add it.
			*/
			agent_conflicts.push_back(c);
		}
	}
}


void MAPF::SimpleBlocking(){
	for (unsigned int i = 0; i < players.size(); i++){ // This is to traverse through the players
		Node destination = players[i].getDestination();
		
		for (unsigned int j = 0; j < paths.size(); j++){ // This is to traverse the paths
			if (i != j){
				unsigned int timeOcurrance = 0;
				if (DetectBlockingHelper(i, j, destination, &timeOcurrance)){			
					Conflicted c;
					//The first player to be added is the player that needs to move
					c.agents.push_back(players[i].getId());
					c.agents.push_back(players[j].getId());

					// The locations of the elements that will be needed for the creation of the submap
					c.locations.push_back(destination.getLocation()); // First the blocking element
					c.locations.push_back(paths[j][timeOcurrance + 1].getLocation()); // Second the blocked element

					c.times.push_back(timeOcurrance + 1); /* We add 1 because a node at index i, ocurrs at time i + 1*/

					if (map->adjacentHelper(destination.getLocation()).size() > 2){
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
	players[index].MoveToClosestEscapeElement(true, paths[index][paths[index].size() -1].getLocation());

	//Now that we've got an escape route, go back to my own destination
	players[index].executeTimeSpaceAstarFromLastIndex();

	//Update the paths
	paths[index] = players[index].getPath();

}

void MAPF::SolveBlockingComplex(Conflicted c){
	// The type of blocking element is detected, now lets solve it
	int indexToMove = getIndexOfAgent(c.agents[0]);
	int indexOther = getIndexOfAgent(c.agents[1]);


	// References to make life easier
	Agent &toMove = players[indexToMove];
	Agent &otherAgent = players[indexOther];

	Location exchange_rate;
	/*
		Let's use the escape Astar v2 to get the closest element to move, but that doesn´t take part of the
		other elements route.
		Firstly, let's compare both the locations of the elements in order to see where is the other element going
		in relationship to the blocking element.
	*/	
	Location blockingLocation = toMove.getDestination().getLocation();
	Location otherLocation = otherAgent.getActualLocation().getLocation();
	
	/*
		Lets see if it is lower than or bigger than the element
	*/
	bool lowerThan = false;
	if (otherLocation < blockingLocation) lowerThan = true;
	
	// We pass that parameter so that the method can bring up the correct escape element
	Node escape = toMove.GetEscapeNodeNotOnRoute(toMove.getDestinationLocation(), otherAgent.getPath(), lowerThan);

	// Now build the submap
	Map submap = map->createSubMap(c.locations[0], escape.getLocation(), c.locations[1], &exchange_rate);

	
	/*
		Fix: We are going to revise the actual value of time, so if it is bigger than 0,
		it means the pathfinding already started, and that we need to modify some parts of the route
		after the time that has passed.
		Date: 22/06/2015
		Why?
		Because if we keep modifying it when the element enters the critical zone, we will modify elements
		on time that already passed, and that is wrong. Very wrong.
	*/
	// Get at what time does the element enters the submap (And the location)
	int time_index = -1;

	if (time > 0){
		// The for loop will start at the current time
		for (unsigned int i = time - 1; i < paths[indexOther].size(); i++){
			// If the element of the map at that location is -1, we find the danger zone
			if (map->getValueAt(paths[indexOther][i].getLocation()) == -1){
				time_index = i;
				break;
			}
		}

		if (time_index == -1){
			time_index = time - 1;
		}
	}
	else {
		for (unsigned int i = 0; i < paths[indexOther].size(); i++){
			// If the element of the map at that location is -1, we find the danger zone
			if (map->getValueAt(paths[indexOther][i].getLocation()) == -1){
				time_index = i;
				break;
			}
		}

		/*
		If the time index is still -1, it means that the submap covers the whole map!
		Therefore the starting index will be 0
		*/
		if (time_index == -1){
			time_index = 0;
		}

		/*
		If the route of the element that is about to move is less than the route of the other element,
		let's make it wait until the element reaches the critical zone
		*/

		if (toMove.pathSize() < time_index + 1){
			while (toMove.pathSize() < time_index + 1){
				toMove.PushElementAtTheBackOfRoute(toMove.getDestination());
			}

			paths[indexToMove] = toMove.getPath();
		}
	}

	// The rest of this code is the same for both types of corrections

	// Now we need to get the index where the element is out of the danger zone
	int exit_index = -1;
	for (unsigned int i = time_index; i < paths[indexOther].size(); i++){
		if (map->getValueAt(paths[indexOther][i].getLocation()) != -1){
			exit_index = i - 1;
			break;
		}
	}
	/*
		Again, if the exit index is -1 it means that the submap is the whole map,
		or the destination of the entity is part of the submap, that is why the exit index
		will be the last element of the path
	*/
	if (exit_index == -1){
		exit_index = paths[indexOther].size() - 1;
	}

	//clean the map of the -1
	map->cleanMap();

	//Now we need to change from normal map coordinates, to submap coordinates
	Location agentLocation1 = paths[indexOther][time_index].getLocation();
	agentLocation1.x = agentLocation1.x - exchange_rate.x;
	agentLocation1.y = agentLocation1.y - exchange_rate.y;

	Location agentExitLocation = paths[indexOther][exit_index].getLocation();
	agentExitLocation.x = agentExitLocation.x - exchange_rate.x;
	agentExitLocation.y = agentExitLocation.y - exchange_rate.y;

	Location agentLocation2 = c.locations[0];
	agentLocation2.x = agentLocation2.x - exchange_rate.x;
	agentLocation2.y = agentLocation2.y - exchange_rate.y;

	/*
		Fix: To avoid any class of head to head collisions, first we are going to calculate the route of one of the elements
		to its escape node, and the other element will just wait on its place. Afterwards, we will use CBS to pathfind to the destination
		of both elements.
		Date: 05/07/2015
		Why?
		Because CBS doesnt has the ability to solve head to head collisions. This will be done in parts to help solve that. If there are no head to
		head collisions, still it will 'look' like an agent is 'waiting' for the other one to finish moving.
	*/

	//Create a CBS with agents
	//First change the escape node to the new location
	escape.ConvertToSubmapCoordinates(exchange_rate);
	Agent partialAgent1(Node(0, 0, agentLocation1.x, agentLocation1.y), Node(0, 0, agentLocation1.x, agentLocation1.y), &submap, 0, 5); // Let that element stay where it is
	Agent partialAgent2(Node(0, 0, agentLocation2.x, agentLocation2.y), escape, &submap, 1, 5);// Escape node is the final destination for this CBS
	
	// Run the CBS only for these agents
	vector<Agent> agents;
	agents.push_back(partialAgent1);
	agents.push_back(partialAgent2);

	//Run the CBS for these situation
	RunCBSUsingPlayers(agents);

	// Get the partial paths
	vector<Node> escapePath1 = tree->getSolution()->getPathAt(0);
	vector<Node> escapePath2 = tree->getSolution()->getPathAt(1);

	/*
		There is one path that only contains one element (since the starting is also the finish destination),
		for this element, we will repeat this step the amount of times the other agent takes to reach its final destination.
	*/
	if (escapePath1.size() < escapePath2.size()){
		for (unsigned int i = 0; i < escapePath2.size(); i++){
			escapePath1.push_back(escapePath1[0]);
		}
	}
	else if (escapePath1.size() > escapePath2.size()){
		while (escapePath1.size() > escapePath2.size()){
			escapePath1.pop_back();
		}
	}
	

	// Both routes are supposed to be of the same size, so we will transform the coordinates to correct coordinates
	for (unsigned int i = 0; i < escapePath1.size(); i++){
		escapePath1[i].ConvertToMapCoordinates(exchange_rate);
		escapePath2[i].ConvertToMapCoordinates(exchange_rate);
	}
	Location partialLocation1 = escapePath1[escapePath1.size() - 1].getLocation();
	Location partialLocation2 = escapePath2[escapePath2.size() - 1].getLocation();

	// Now we have 2 partial routes, it is time to calculate the routes from their partial destination, to their actual destination
	//Now we can create a CBS with agents
	//This will be calculated from partial location to their actual destination (or exit stuff)
	Agent agent1(Node(0,partialLocation1), Node(0, 0, agentExitLocation.x, agentExitLocation.y), &submap, 0, 5);
	Agent agent2(Node(0,partialLocation2), Node(0, 0, agentLocation2.x, agentLocation2.y), &submap, 1, 5);
	agents.clear(); // Clear from the old agents
	agents.push_back(agent1);
	agents.push_back(agent2);

	//Run the CBS
 	RunCBSUsingPlayers(agents);

	// Once the paths have been calculated, now we can update the paths of the agents (converting them to normal coordinates)
	vector<Node> new_path;
	// First get all the elements before the agent got into the danger zone
	for (int i = 0; i <= time_index; i++){
		new_path.push_back(paths[indexOther][i]);
	}

	// Now, get the elements calculated by CBS
	vector<Node> path1 = tree->getSolution()->getPathAt(0);
	vector<Node> path2 = tree->getSolution()->getPathAt(1);

	// Now, update paths with the elements of the escape routes
	for (unsigned int i = 0; i < escapePath1.size(); i++){
		new_path.push_back(escapePath1[i]);
		toMove.PushElementAtTheBackOfRoute(escapePath2[i]);
	}

	/*	
		Now we transform the elements of the path from submap coordinates, to global coordinates
		And we update the paths of the other elements
	*/
	for (unsigned int i = 0; i < path1.size(); i++){
		path1[i].ConvertToMapCoordinates(exchange_rate);
		new_path.push_back(path1[i]);
	}

	for (unsigned int i = 0; i < path2.size(); i++){
		path2[i].ConvertToMapCoordinates(exchange_rate);
		toMove.PushElementAtTheBackOfRoute(path2[i]);
	}

	// Finish updating the path of the element that is fololowing its path
	for (unsigned int i = exit_index; i < otherAgent.pathSize(); i++){
		new_path.push_back(paths[indexOther][i]);
	}

	otherAgent.setPath(new_path); //Update the path directly to the agent
	
	//Update the paths
	paths[indexOther] = otherAgent.getPath();
	paths[indexToMove] = toMove.getPath();
}

bool MAPF::existsInList(vector<int> list, int val){
	if (std::find(list.begin(), list.end(), val) != list.end()) return true;

	return false;
}

bool MAPF::NodeExistsOnList(vector<Node> list, Node val){
	if (std::find(list.begin(), list.end(), val) != list.end()) return true;

	return false;
}

// A location was used to save memory
int MAPF::GetIndexAtArray(vector<Node> list, Location val){
	int result = -1;
	for (unsigned int i = 0; i < list.size(); i++){
		if (val == list[i].getLocation()){
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