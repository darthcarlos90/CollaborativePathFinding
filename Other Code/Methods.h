/*
	Here I will save all the methods that will not be used on the project, 
	but that are worth taking a look.
*/

// Method that detects deadlocks
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

// This method solves the above using CBS
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
		int id = root->addAgent(players[index]); // For later use
		root->AddPath(players[index].getPath());
		root->addAgent(players[c.agents[1]]);
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

/*
	This method was created to solve simple cases, but, it is not used any more.
*/
void MAPF::ConflictSolver(Conflicted c){
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
	Agent agent1(Node(0, partialLocation1), Node(0, 0, agentExitLocation.x, agentExitLocation.y), &submap, 0, 5);
	Agent agent2(Node(0, partialLocation2), Node(0, 0, agentLocation2.x, agentLocation2.y), &submap, 1, 5);
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

// Step by step of what to do when there is a blocking element
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

/*
	Monster method that describes how to solve really complex problems
	This is the base for the method used on the actual solution.
*/
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

/*
	Creation of submaps based on the escape nodes and the locations of the elements
*/
Map Map::createSubMap(Location blocking, Location escape, Location blocked, Location* difference){
	
	/*
	Get the row where the blocking element is.
	*/
	Location lowerBounds = blocking;
	Location upperBounds = blocking;

	/*
		If the escape element is on the same row as the blocking element, then
		grab the upper and lower rows.
	*/
	if (blocking.y == escape.y){
		if(lowerBounds.y > 0) 
			lowerBounds.y = lowerBounds.y - 1;
		if(upperBounds.y < data->get_y_size() - 1)
			upperBounds.y = upperBounds.y + 1;
	} //Otherwise, get the row where the escape element is
	else {
		if (lowerBounds.y > escape.y) 
			lowerBounds.y = escape.y;
		else if (upperBounds.y < escape.y) 
			upperBounds.y = escape.y;
	}

	/*
		Get the elements that are on the furthest side of the 
	*/
	if (lowerBounds.x > escape.x) 
		lowerBounds.x = escape.x;
	if (upperBounds.x < escape.x) 
		upperBounds.x = escape.x;
	if (lowerBounds.x > blocked.x) 
		lowerBounds.x = blocked.x;
	if (upperBounds.x < blocked.x) 
		upperBounds.x = blocked.x;
	
	//Finally, add the elements if they are outside our danger zone
	if (lowerBounds.y > blocked.y) lowerBounds.y = blocked.y;
	if (upperBounds.y < blocked.y) upperBounds.y = blocked.y;

	/*
		There are some situations, where leaving the exact size of the submap may 
		cause deadlocks to happen. To make solution simpler, the map will be
		expanded a bit.
	*/

	if(upperBounds.x < data->get_x_size() - 1)upperBounds.x = upperBounds.x + 1;
	if(upperBounds.y < data->get_y_size() - 1)upperBounds.y = upperBounds.y + 1;
	if(lowerBounds.x > 0) lowerBounds.x = lowerBounds.x - 1;
	if(lowerBounds.y > 0)lowerBounds.y = lowerBounds.y - 1;

	//Now that we have the bounds of our submap, we can create the matrix with the data
	Matrix<int> *subdata = new Matrix<int>();
	*subdata = getSubData(lowerBounds.x, lowerBounds.y, upperBounds.x, upperBounds.y);

	//Now we can return a map based on the subdata
	Map submap(subdata);

	subdata = NULL;

	// If there is a pointer, lets feed it data
	if (difference){
		*difference = lowerBounds;
	}

	//return the result
	return submap;
}