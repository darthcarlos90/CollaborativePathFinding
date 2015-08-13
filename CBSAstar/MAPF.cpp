#include "MAPF.h"

//Constructor that loads data from the file
MAPF::MAPF(string filename){
	//Loading data from the file
	fr = new FileManager(filename, true);

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
	
	algorithm_type = 0;
	

}

//Constructor that asks the user what are the specifications of the grid and randomly creates it
MAPF::MAPF(int size_x, int size_y, int max_players){
	// Create the map of the size selected
	map = new Map(size_x, size_y);
	broken = false;
	int D = map->CalculateD();
	
	int n_players = max_players;
	//if (max_players > 2) n_players = std::rand() % 2 + max_players;

	// We now have an emty map, next step is create the agents and set them goals
	for (int i = 0; i < n_players; i++){
		Location start_location(std::rand() % size_x, std::rand() % size_y);
		while (map->getValueAt(start_location) != 0){
			start_location = Location(std::rand() % size_x, std::rand() % size_y);
		}
		Location end_location(std::rand() % size_x, std::rand() % size_y);
		while (map->getValueAt(end_location) != 0 || end_location == start_location){
			end_location = Location(std::rand() % size_x, std::rand() % size_y);
		}

		Node start_node(i + 2, start_location);
		Node goal_node(i + 2, end_location);

		Agent agent(start_node, goal_node, map, i + 2, D);

		map->setElement(start_location, i + 2);
		map->setElement(end_location, i + 2);

		players.push_back(agent);
		
	}
	int biggerManhattan = 0;
	for (unsigned int i = 0; i < players.size(); i++){
		if (players[i].getManhattanBetweenNodes() > biggerManhattan)
			biggerManhattan = players[i].getManhattanBetweenNodes();
	}
	cout << "Number of players: " << n_players << endl;
	// Once that the random players where created, it is time to create a random number of obstacles
	do{
		// Add obstacles to the map
		AddMapObstacles(biggerManhattan);
		// if it is an invalid map, create a new one
	} while (!ValidMap());
	resetEntities(); // to clear all the paths
	system("cls"); 
	map->printData();

	time = 0;
	algorithm_type = 0;

}


MAPF::~MAPF(void){
	delete fr;
	delete map;
	//Dont delete the nodes, thats the tree's job
	delete tree;
}


void MAPF::addRandomPlayer(){
	Location start_location(std::rand() % map->getXValue(), std::rand() % map->getYValue());
	while (map->getValueAt(start_location) != 0){
		start_location = Location(std::rand() % map->getXValue(), std::rand() % map->getYValue());
	}
	Location end_location(std::rand() % map->getXValue(), std::rand() % map->getYValue());
	while (map->getValueAt(end_location) != 0){
		end_location = Location(std::rand() % map->getXValue(), std::rand() % map->getYValue());
	}

	Node start_node(players.size() + 1, start_location);
	Node goal_node(players.size() + 1, end_location);

	Agent agent(start_node, goal_node, map, players.size() + 1, map->CalculateD());

	map->setElement(start_location, players.size() + 1);
	map->setElement(end_location, players.size() + 1);

	players.push_back(agent);
}

void MAPF::PrintPlayers(ostream &out){
	for (unsigned int i = 0; i < players.size(); i++){
		out << "Player " << players[i].getId() << ": " << endl;
		out << "Start Location: (" << players[i].getStartLocation().x << ", " << players[i].getStartLocation().y << ")" << endl;
		out << "Destination: (" << players[i].getDestinationLocation().x << ", " << players[i].getDestinationLocation().y << ")" << endl;
	}
	out << endl;
}

void MAPF::PrintPaths(ostream& out){
	for (unsigned int i = 0; i < players.size(); i++){
		if (players[i].hasValidSolution()){
			out << "Agent's " << players[i].getId() << " path:" << endl;
			for (unsigned int j = 0; j <players[i].pathSize(); j++){
				out << "t" << j << ": ( " << 
					players[i].getPath()[j].getX() << ", " << 
					players[i].getPath()[j].getY() << ")" << endl;
			}
			
		}
		else {
			out << "Agent " << players[i].getId() << " has an invalid path" << endl;
		}

		out << endl;
		
	}
}

void MAPF::Start(int type){
	algorithm_type = type;
	cout << "Number of players: " << players.size() << endl;
	if (!broken){
		cout << "Calculating Routes" << endl;
		switch (type){
		case 1: // 1 means normal silvers calculations
			StartSilversPathFinding();
			break;
		case 2: // 2 means cbs calculation
			StartCBSPathFinding();
			break;
		case 3: // 3 means hybrid
			StartHybridPathFinding();
			break;
		}
	}
	else {
		cout << "Broken file, exiting...." << endl;
	}
}

void MAPF::StartSilversPathFinding(){
	for (unsigned int i = 0; i < players.size(); i++){
		players[i].executeTimeSpaceAstar(0); // because element 0 is the starting position
		paths.push_back(players[i].getPath());
	}
}

void MAPF::StartCBSPathFinding(){
	
	if (RunCBSUsingPlayers(players)){
		for (unsigned int i = 0; i < players.size(); i++){
			players[i].setPath(tree->getSolution()->getPathAt(i));
			paths.push_back(players[i].getPath());
			players[i].setValidPath(true);
		}
	}

}

void MAPF::StartHybridPathFinding(){
	// Do pathfinding as silvers would do
	StartSilversPathFinding();
	// Now check for any inconsistency
	RevisePaths();
}

bool MAPF::CBSHelper(){
	bool solutionFound = false;
	//While we can't find the solution
	while (!solutionFound){
		//Get the best node of the tree
		CBTNode* P = tree->getSolution();
		if (P->isValidNode()){
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
		else{
			// There was not a solution found
			break;
		}
	}

	return solutionFound;
}

bool MAPF::RunCBSUsingPlayers(vector<Agent> agents){
	//Create the constraint tree
	tree = new ConstraintTree();

	//Create the root node
	root = new CBTNode();

	//Add the agents to the root node
	for (unsigned int i = 0; i < agents.size(); i++){
		root->addAgent(agents[i]);
	}

	//find the individual paths of the elements
	root->CalculatePaths();

	//calculate the cost of this node
	root->calculateCost();

	//insert the root into the tree
	tree->insertRoot(root);
	
	return CBSHelper();
}

void MAPF::MoveEntities( bool automatic){
	switch (algorithm_type){
	case 1:
		MoveBySilvers(false, automatic);
		break;
	case 2:
		MoveByCBS(automatic);
		break;
	case 3:
		MoveBySilvers(true, automatic);
		break;
	}
}



void MAPF::MoveBySilvers(bool hybrid, bool automatic){
	time = 1;
	bool finished = false;
	bool verify = false;
	while (!finished){
		if (!automatic)system("cls");
		if (!automatic) map->cleanMap();
		finished = players[0].finished();
		for (unsigned int i = 0; i < players.size(); i++){
			if (players[i].hasValidSolution()){
				players[i].move(time); // The element at i will be the element at time = i + 1
				if (i > 0) finished = finished && players[i].finished();
				verify = verify || players[i].NeedsPathVerification(); // When An element just updated its path
				map->setElement(players[i].getLocation(), (players[i].getId() + 2)); // Uncoment for clean whatever
			}
			
		}
		// If an element just changed it's path, verify it
		if (verify){
			verify = false;
			// Update the paths on the lists
			for (unsigned int i = 0; i < players.size(); i++){
				paths[i] = players[i].getPath();
				players[i].SetPathVerificationFlag(false);
			}
			//Now revise the paths if hybrid movement
			if(hybrid)RevisePaths();
		}

		time++;
		if (!automatic)map->printData();
		if(!automatic)system("pause");
	}

	if (!automatic){
		for (unsigned int i = 0; i < players.size(); i++){
			cout << "Path of agent " << players[i].getId() << endl;
			for (unsigned int j = 0; j < paths[i].size(); j++){
				cout << "t" << j << ": ";
				paths[i][j].printValue();
			}
		}
	}

}

void MAPF::MoveByCBS(bool automatic){
	bool finished = false;
	if (paths.size() == players.size()){
		while (!finished){
			if (!automatic)system("cls");
			if (!automatic)map->cleanMap();
			for (unsigned int i = 0; i < players.size(); i++){
				players[i].moveEntity(time);
				map->setElement(players[i].getX(), players[i].getY(), (players[i].getId() + 2));
			}
			time++;

			finished = players[0].finished();
			for (unsigned int i = 1; i < players.size(); i++){
				finished = finished && players[i].finished();
			}
			if (!automatic)map->printData();

			if (!automatic)system("pause");
		}

		if (!automatic){
			for (unsigned int i = 0; i < players.size(); i++){
				cout << "Path of agent " << players[i].getId() << endl;
				for (unsigned int j = 0; j < paths[i].size(); j++){
					cout << "t" << j << ": ";
					paths[i][j].printValue();
				}
			}
		}

	}
	else {
		cout << "SOLUTION NOT FOUND." << endl;
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
		ConflictSolver(c);
	}
	
}


void MAPF::ConflictSolver(Conflicted c){
	// The type of blocking element is detected, now lets solve it
	int indexToMove = getIndexOfAgent(c.agents[0]);
	int indexOther = getIndexOfAgent(c.agents[1]);


	// References to make life easier
	Agent &toMove = players[indexToMove];
	Agent &otherAgent = players[indexOther];

	// This wll be used to transform coordinates from bbig map to small map and vicecersa
	Location exchange_rate;
	

	// Let's build the submap
	Map submap = map->createSubMap(c.locations, &exchange_rate);


	
	// Get at what step does the element enters the submap (And the location)
	int time_index = -1;

	if (time > 0){
		// The for loop will start at the current time
		for (unsigned int i = time; i < paths[indexOther].size(); i++){
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

	}

	// Now we need to get the index where the element is out of the submap zone
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

	
	Agent partialAgent1(Node(0, agentLocation1), Node(0,agentExitLocation), &submap, 0, 5); // Move to the exit location
	Agent partialAgent2(Node(0, agentLocation2), toMove.getDestination(), &submap, 1, 5); // Move to its destination, let CBS do the magic

	// Run the CBS only for these agents
	vector<Agent> agents;
	agents.push_back(partialAgent1);
	agents.push_back(partialAgent2);

	//Run the CBS for these situation
	bool solutionFound = false;
	//TODO: Continue this
	while (!solutionFound){
		RunCBSUsingPlayers(agents);
	}
		

	// Get the partial paths
	vector<Node> escapePath1 = tree->getSolution()->getPathAt(0);
	vector<Node> escapePath2 = tree->getSolution()->getPathAt(1);


	// Both routes are supposed to be of the same size, so we will transform the coordinates to correct coordinates
	for (unsigned int i = 0; i < escapePath1.size(); i++){
		escapePath1[i].ConvertToMapCoordinates(exchange_rate);
		escapePath2[i].ConvertToMapCoordinates(exchange_rate);
	}
	

	// Once the paths have been calculated, now we can update the paths of the agents (converting them to normal coordinates)
	vector<Node> new_path;
	
	// First get all the elements before the agent got into the danger zone
	for (int i = 0; i <= time_index; i++){
		new_path.push_back(paths[indexOther][i]);
	}

	// Now, update paths with the elements of the escape routes
	for (unsigned int i = 0; i < escapePath1.size(); i++){
		new_path.push_back(escapePath1[i]);
		toMove.PushElementAtTheBackOfRoute(escapePath2[i]);
	}

	// Finish updating the path of the element that is following its path
	for (unsigned int i = exit_index; i < otherAgent.pathSize(); i++){
		new_path.push_back(paths[indexOther][i]);
	}

	otherAgent.setPath(new_path); //Update the path directly to the agent

	//Update the paths
	paths[indexOther] = otherAgent.getPath();
	paths[indexToMove] = toMove.getPath();
}

/*
	Method that looks if vector b is a subset of vector a.
*/
bool MAPF::IsSubset(vector<int> a, vector<int> b){
	std::sort(a.begin(), a.end());
	std::sort(b.begin(), b.end());
	return std::includes(a.begin(), a.end(), b.begin(), b.end());
}

bool MAPF::AlreadyOnConflict(vector<int> agents, int type){
	bool result = false;
	for (unsigned int i = 0; i < agent_conflicts.size(); i++){
		if (agent_conflicts[i].type == type){
			if (IsSubset(agent_conflicts[i].agents, agents)){
				result = true;
				break;
			}
		}
		
	}

	return result;
}


// This method simply calls the helper methods for blocking
void MAPF::Blocking(){
	MultipleBlocking();
	SimpleBlocking();
}

// A helper function for the helper functions. 
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
	// TODO: Debug this
	for (unsigned int i = 0; i < players.size(); i++){ // This for is to see through the players
		Node destination = players[i].getDestination();
		int blockedElements = 0;
		Conflicted c;
		for (unsigned int j = 0; j < paths.size(); j++){
			if (i != j){
				if (DetectBlockingHelper(i, j, destination)){
					blockedElements++;
					c.agents.push_back(players[j].getId());
				}
			}
		}
		// If this element is blocking various paths
		if (blockedElements > 1){
			/*
			After a multiple block has been identified, now I need to check if this multiple block is not part of a bigger multiple block.
			If it is already part of a multiple block, dont add it to the conflicts, otherwise add it.
			*/
			bool conflictRegistered = false;
			if (!AlreadyOnConflict(c.agents, BLOCKING_MULTIPLE)){
				c.type = BLOCKING_MULTIPLE;
				c.agents.push_back(players[i].getId());
				agent_conflicts.push_back(c);
			}
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

					// Now, check if this simple blocking is not just part of a multiple blocking
					if (!AlreadyOnConflict(c.agents, BLOCKING_MULTIPLE)){
						// The locations of the elements that will be needed for the creation of the submap
						c.locations.push_back(destination.getLocation()); // First the blocking element
						if (timeOcurrance + 1 < paths[j].size()){
							c.locations.push_back(paths[j][timeOcurrance + 1].getLocation()); // Second the blocked element
						}
						else c.locations.push_back(destination.getLocation());




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

void MAPF::AddMapObstacles(int limit){
	int size_x = map->getXValue();
	int size_y = map->getYValue();
	
	map->cleanObstacles();

	int upperObstacleLimit = (size_x * size_y) - limit;
	int obstacleNumber = std::rand() % upperObstacleLimit;

	for (int i = 0; i < obstacleNumber; i++){
		// Get a random location
		Location obstacleLocation(std::rand() % size_x, std::rand() % size_y);

		// See if it is a valuable location
		while (map->getValueAt(obstacleLocation) != 0){
			// If not, change it until a valuable location is found
			obstacleLocation = Location(std::rand() % size_x, std::rand() % size_y);
		}

		// Set that location to an obstacle
		map->setElement(obstacleLocation, 1);
	}

}

/*
	This mehod checks if it is a valid map.
	It checks if it's elements can reach it's destination.
	If they can't, then there is an invalid map.
*/
bool MAPF::ValidMap(){
	// Calculate the sime route of an element
	for (unsigned int i = 0; i < players.size(); i++){
		players[i].calculateRoute();
	}

	// Check if all the elements reached their destination
	bool result = (players[0].getDestination() == players[0].getPath()[players[0].pathSize() -1]);
	for (unsigned int i = 1; i < players.size(); i++){
		if (!result) break;
		result = (players[i].getDestination() == players[i].getPath()[players[i].pathSize() - 1]);
	}

	

	return result;
}

void MAPF::printCosts(ostream& out){
	for (unsigned int i = 0; i < players.size(); i++){
		out << "Player " << players[i].getId() << " cost: " << players[i].getSic() << endl;
	}
	int total_cost = 0;
	switch (algorithm_type){
	case 1:
	case 3:
		for(unsigned int i = 0; i < players.size(); i++){
			total_cost += players[i].getSic();
		}
		out << "Total cost: " << total_cost << endl;
		break;
	case 2:
		out << "Total cost: " << tree->getSolution()->getCost() << endl;
		break;
	}
}

/*
	Takes all the agents back to their starting position, and
	clears the paths.
*/
void MAPF::resetEntities(){
	paths.clear();
	for (unsigned int i = 0; i < players.size(); i++){
		players[i].resetElement();
	}
	time = 0;
}

void MAPF::cleanReservationsConstraints(){
	map->cleanConstraintsReservations();
}