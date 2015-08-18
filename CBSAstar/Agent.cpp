#include "Agent.h"


Agent::Agent(Node location,Node destination, Map* m, int id, unsigned int D):
actualNode(location), startingPoint(location)
{
	map = m;
	active = true;
	this->id = id;
	this->steps_limit = D;
	this->destination = destination;
	SIC = 0;
	map->setElement(actualNode.getX(), actualNode.getY(), id + 2);
	map->setElement(destination.getX(), destination.getY(), id + 2);
	system("cls");
	map->printData();
	needsPathVerification = false;
	index_lower_spatial_openList = 0;
	index_lower_time_openList = 0;
	validSolution = false;
	priority = false;
	last_reserved = 0;
}

Agent::~Agent(void){
	map = NULL;//There is only one map, it is not my job to delete it
}


bool Agent::ConstraintAstar(Location start, Location finish, int starting_time, vector<Constraint> constraints){
	// Since start is at time starting_time - 1, get nodes for starting_time
	Node A(0, start);
	bool pathFound = false;
	
	vector<Node> adjacents = getAdjacents(A, finish);

	// Add them to the adjacent list
	for (unsigned int i = 0; i < adjacents.size(); i++){
		AddToTimedSpatialOpenListCAT(adjacents[i]);
	}
	
	// Start with those nodes
	Node P;
	//If the goal was found, break
	while (!pathFound){

		// If the open list is empty, no path was found, break
		if (spatial_openList.size() == 0) 
			break;

		//let p be the best node in the open list
		P = spatial_openList[index_lower_spatial_openList];
		UpdateSpatialOpenListCAT();
		//Some changes

		// If P is a valid movement, it will be processed, otherwise, it will be discarded
		int timespan = (P.getDepth() - 1) + starting_time;
		if (validMovement(P.getLocation(),timespan , constraints)){
			spatial_closedList.push_back(P);


			if (P.getLocation() == finish){
				if (!FindSpecialCaseCBS(finish, timespan, constraints)){
					pathFound = true;
					break;
				}	
			}

			vector<Node> adjacents = getAdjacents(P, finish);
			for (unsigned int i = 0; i < adjacents.size(); i++){
				AddToTimedSpatialOpenListCAT(adjacents[i]);
			}
		}
		
	}

	// If the path was found
	if (pathFound){
		vector<Node> inverse_route;
		//Once the path has been found, retrace your steps
		if (P.hasParent()){
			while (P.hasParent()){
				inverse_route.push_back(P);
				P = P.getParent();
			}
		}
		else inverse_route.push_back(P); //This in case the next node is the answer

		//For some reason the route is backwards, lets but it on the corect order
		for (int i = inverse_route.size() - 1; i >= 0; i--){
			// Remove if something breaks
			inverse_route[i].clearParent(); // To save memory
			spatial_route.push_back(inverse_route[i]);// Save it on the spatial_route
		}

	}
	
	return pathFound;
}


/*
Astar algorithm built from the Tutorial 2 on AI and from the
video at this link: https ://www.youtube.com/watch?v=KNXfSOx4eEE

*/
void Agent::executeSpatialAstar(Location start, Location finish){

	bool pathFound = false;
	//Let A be the starting point
	Node A (0, start);
	// Assign f, g and h values to A
	A.setG(0);
	A.calculateManhattanHeuristic(finish);
	A.calculateF();

	//Add A to the open list, At this point, A shoul be the only node on the open list
	spatial_openList.push_back(A);
	index_lower_spatial_openList = 0;

	Node P;
	//If the goal was found, break
	while (!pathFound){

		// If the open list is empty, no path was found, break
		if (spatial_openList.size() == 0)
			break;

		//let p be the best node in the open list
		/*	Since the openList is sorted every time an item is added, then the best
			option to select is the first item
		*/
		P = spatial_openList[index_lower_spatial_openList];
		spatial_closedList.push_back(P);
		UpdateSpatialOpenList();

		/*
			Fix:The comparison here was changed, instead to see if the type is of 3, we compare against
			the final node.
			Date: 01/05/2015
			Why?
			Because originally it was comparing against 3, but because we are on time-space Astar, that
			methodology is used no more, so now we compare to see if it is the destination node.
		*/
		if (P.getLocation() == finish){
			pathFound = true;
			break;
		}

		vector<Node> adjacents = getAdjacents(P, finish);
		for (unsigned int i = 0; i < adjacents.size(); i++){
			addToSpatialOpenList(adjacents[i]);
		}
		/*
			Fix: because the node class is too "heavy" to be sorted, it will be changed for a 
			normal linear search.
			Date: 23/06/2015
			Why?
			Because when sorting, it involves calling too much of the copy constructor,
			specially in really large lists. with this, and with the parent elements, it
			takes too much time to sort a list of nodes.
		*/
		// If some stuff stops working, go back to the original way
		//std::sort(spatial_openList.begin(), spatial_openList.end());
	}

	if (pathFound){
		vector<Node> inverse_route;
		//Once the path has been found, retrace your steps
		//This is not addding the first element of the route
		if (P.hasParent()){
			while (P.hasParent()){
				inverse_route.push_back(P);
				P = P.getParent();
			}
		}
		else inverse_route.push_back(P); //This in case the next node is the answer
		// That is why here, we will add the very first element of the route to the inverse route
		inverse_route.push_back(A);

		//For some reason the route is backwards, lets but it on the corect order
		for (int i = inverse_route.size() - 1; i >= 0; i--){
			// Remove if something breaks
			inverse_route[i].clearParent(); // To save memory
			spatial_route.push_back(inverse_route[i]);
		}

	}

	validSolution = pathFound;
}

int Agent::executebacksearchAstar(Location start, Location finish){
	// clear open list
	//spatial_openList.clear();
	
	bool nodeFound = false;
	//Let A be the starting point
	Node A(0, start);
	// Assign f, g and h values to A
	A.setG(0);
	A.calculateManhattanHeuristic(finish);
	A.calculateF();

	//Add A to the open list, At this point, A shoul be the only node on the open list
	spatial_openList.push_back(A);
	index_lower_spatial_openList = 0;

	Node P;
	//If the goal was found, break
	while (!nodeFound){

		// If the open list is empty, no path was found, break
		if (spatial_openList.size() == 0)
			break;

		//let p be the best node in the open list
		P = spatial_openList[index_lower_spatial_openList];
		spatial_closedList.push_back(P);
		UpdateSpatialOpenList();

		if (P.getLocation() == finish){
			nodeFound = true;
			break;
		}

		vector<Node> adjacents = getAdjacents(P, finish);
		for (unsigned int i = 0; i < adjacents.size(); i++){
			addToSpatialOpenList(adjacents[i]);
		}
	}

	return P.getG();
	
}

// Move method used by Silvers and Hybrid
void Agent::move(unsigned int t){
	/*
	Step 3: Every t, advance one spot in the road.
	- If d has been reached or we reaced to the end of the road, stop;
		otherwise, continue advancing.
	*/
	if (actualNode == destination && t >= time_route.size()) {
		active = false; //we reached the end of the route 
	}
	else {
		actualNode = time_route[t];
		stepsTaken++;
	}
	

	cout << "Unit: " << id + 2 << " at location: " << actualNode.getX() << " , " << actualNode.getY();
	if (!active) cout << " This element is finished.";
	cout << endl;
	
	/*
		Step 4: If the d/2 has been reached, and the last step doesn't arrives to the destination,
		then we need to plan the rest of the route
	*/
	if(stepsTaken >= (steps_limit / 2) && time_route[time_route.size() - 1] != destination){
		/*
			This are cleared because the g, h, and f values of the nodes have changed, since now we
			are repathing from the perspective of the actualNode, not the beggining node.
		*/
		time_route[time_route.size() -1].clearParent(); //Clear the parent, so this is a true starting point
		stepsTaken = 0;
		int new_starting_time = time_route.size();
		TimeSpaceAstarHelper(
			time_route[time_route.size() - 1].getLocation(), // The las node on the path
			destination.getLocation(), // The destination of the pathfinding
			new_starting_time); // So that the reroute starts at the end of the route
		reserveRouteFromIndex(new_starting_time); // The index will be the last index of the table
		needsPathVerification = true;
	}
	
}


/*
	This algorithm was created based on the work done by David Silver in his paper
	named Cooperative Pathfinding. Some other code was also added by myself.
	Params:
		- starting_time: The time at which we start the pathfinding, so that the next step
						in the route of the element will be time + 1
*/
void Agent::executeTimeSpaceAstar(int starting_time){
	
	/*
		Step 1 is described in the function that will execute.
	*/
	TimeSpaceAstarHelper(actualNode.getLocation(), destination.getLocation(), starting_time);
	
	/* 
		Step 2: Once the route has been found, reserve your path.
			- Only reserve the blocks of your path where nPath <= steps
	*/
	// Now that we have a route, reserve it
	// If we dont have a valid solution, dont reserve it
	if (validSolution) {
		reserveRoute(starting_time);

		// Now, if the first element of the time route is not the initial step, add it
		if (time_route[0] != startingPoint){
			vector<Node> temp = time_route;
			time_route.clear();
			time_route.push_back(startingPoint);
			for (unsigned int i = 0; i < temp.size(); i++){
				time_route.push_back(temp[i]);
			}
		}
	}
		
}

/*
	This method is create to execute Time-Space Astar starting from the last 
	element on the time route, till it finds the end (or reaches the limit of
	steps).
*/
void Agent::executeTimeSpaceAstarFromLastIndex(){
	int index = time_route.size();
	TimeSpaceAstarHelper(
		time_route[time_route.size() - 1].getLocation(), 
		destination.getLocation(), 
		time_route.size());
	reserveRouteFromIndex(index);
}

void Agent::TimeSpaceAstarHelper(Location start, Location finish, int time){	
	// In case they're not empty
	time_closedList.clear();
	time_openList.clear();
	
	// The time at which the pathfinding started
	int starting_time = time;
	/*
	Step 1: Calculate the route.
	- Calculate the route using regular Astar, BUT the H used on every node will be
	the G value calculated in the reverse Astar.
	- When calculating the route, check if that element has been reserved by someone
	for some time t, otherwise, it will not be used.
	*/
	bool pathFound = false;
	bool emtyOpenList = false; // flag for partial path empty list
	Node A(0, start); //Let A be the starting point

	A.setG(0);
	calculateRealHeuristic(&A, finish);
	A.calculateF();
	
	time_openList.push_back(A); //We put it on the open list
	index_lower_time_openList = 0;

	Node P;
	

	int step_counter = 0; //This counter will help us find a partial path
	
	// While the path hasn't been found
	while (!pathFound){

		/*
			If the list is empty, means that a path couldn be found for now,
			so lets get a partial path
		*/
		if (time_openList.empty()){
			// invalid solution, break
			break;
		}	 
		
		//let p be the best node in the open list
		P = time_openList[index_lower_time_openList];
		time_closedList.push_back(P);
		UpdateTimeOpenList();

		// If P is the goal node, then we finished
		if (P.getLocation() == finish){
			pathFound = true;
			break;
		}

		//otherwise
		// Get the adjacents of node P
		/*
			Explanation:
			The time_span variable is telling us in what time span look for the adjacents.
			We add 3 variables, the depth() of P, which gives us at what time is the node P
			being used, starting_time, in case the starting time is not 0, in this case, if P 
			has a depth of 0, and the starting time is 7, the time of P is 7, not 0. Finally
			we add 1, since we are looking for nodes to be the next step after P.
		*/
		int time_span = P.getDepth() + starting_time + 1;
		vector<Node> adjacents = getTimedAdjacents(P, time_span, finish);

		// Add the adjacents to the open list
		for (unsigned int i = 0; i < adjacents.size(); i++){
			addToTimeOpenList(adjacents[i]);
		}
		
		// Save in what step whe currently are
		step_counter = P.getDepth() + 1;
		
		// If we reached the step limit
		if ((step_counter == steps_limit) || emtyOpenList){ 
			//Check for any partial path
			for (unsigned int i = 0; i < time_closedList.size(); i++){
				if (time_closedList[i].getDepth() >= steps_limit){
					// A partial path was found
					partial_path_nodes.push_back(time_closedList[i]);
				}
			}
			if (partial_path_nodes.size() > 0){ // If we have a partial path
				pathFound = true;
				break;
			}
		}
	}
		
	//If we have a partial path, populate the route with the path
	if (partial_path_nodes.size() > 0){
		//If we have more than 1 partial path, lets decide using terminal node
		if (partial_path_nodes.size() > 1){
			int smaller_index =  0;
			int smaller_value = 0;

			//Get the element with the smallest value from d + 1 till the end
			for (unsigned int i = 0; i < partial_path_nodes.size(); i++){
				// Get the adjacents of the last node
				vector<Node> around = getTimedAdjacents(partial_path_nodes[i], 
					partial_path_nodes[i].getDepth() + 1, finish); // get the adjacents
				
				clearSpatialLists(true);
				// This option is our terminal node, calculate route from terminal node to end
				executeSpatialAstar(around[0].getLocation(), destination.getLocation());
				
				int route_value = 0;
					
				//Get the value of the route
				for (unsigned int j = 0; j < spatial_route.size(); j++){
					route_value += spatial_route[j].getF();
				}
				
				// save to compare smaller value
				if (i == 0){
					smaller_value = route_value;
				}
				else{
					if (route_value < smaller_value){
						smaller_index = i;
						smaller_value = route_value;
					}
				}
			}

			//Now that we've got the smaller route, we will follow it
			//Set P to be the node of the smalles route
			P = partial_path_nodes[smaller_index];
			partial_path_nodes.clear(); // Eliminate this one so it doesn't gets stuck in an infinite loop
		}
	}
	// If we have a valid solution
	if (pathFound){
		// Now, lets populate the path vector
		vector<Node> temp;

		if (P.hasParent()){
			while (P.hasParent()){
				temp.push_back(P);
				P = P.getParent();
			}
		}
		else temp.push_back(P);

		// The first element of the route
		temp.push_back(A);

		//get the nodes in order
		for (int i = temp.size() - 1; i >= 0; i--){
			// Fix if something brakes
			temp[i].clearParent(); //to save memory on copying parents, since parents are not being used anymore
			time_route.push_back(temp[i]);
		}
	}

	validSolution = pathFound;
}


int Agent::executeSpatialAstarUntilFound(Location start, Node toFind){
	
	if (!FindNodeAtSpatialClosedList(toFind)){
		// If the node we are looking for is not on the closed list
		// look for it
		return executebacksearchAstar(start, toFind.getLocation());
	}
	else{
		int index = GetIndexOfElementAtSpatialClosedList(toFind.getLocation());
		int resutl = spatial_closedList[index].getG();
		return resutl;
	}
	
}

void Agent::calculateRealHeuristic(Node* toCalculate, Location finish){
	toCalculate->setH(executeSpatialAstarUntilFound(finish, *toCalculate));
	//spatial_openList.clear();// That way the search can restart
	index_lower_spatial_openList = 0;
}


vector<Node> Agent::getAdjacents(Node element, Location ending){
	vector<Node> result;

	result = map->adjacentHelper(element.getLocation()); // Use helper function to make life easier

	//calculate heuristic of every element
	for (unsigned int i = 0; i < result.size(); i++){
		result[i].calculateManhattanHeuristic(ending);
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}

	// Sorting must be done here
	std::sort(result.begin(), result.end());

	return result;
}

vector<Node> Agent::getAdjacentsonConstraints(Node element, Location ending, vector<Constraint> constraints, int time){
	vector<Node> temp;
	vector<Node> result;

	// Get the adjacents of the element
	temp = map->adjacentHelper(element.getLocation());
	element.setIndividualG(1);
	element.calculateG();
	temp.push_back(element); // So we can have the option to stay
	for (unsigned int i = 0; i < temp.size(); i++){
		bool found = false;
		// Check the constraint list
		for (unsigned int j = 0; j < constraints.size(); j++){
			// If the id is different
			if (constraints[j].id != id){
				// If the time is the same
				if (constraints[j].t == time){
					// and same location
					if (temp[i].getLocation() == constraints[j].location){
						// It is reserved, break
						found = true;
						break;
					}
				}
			}
		}
		// If it is not on the constraint list
		if (!found){
			//Calculate its stuff
			temp[i].calculateManhattanHeuristic(ending);
			temp[i].setParent(element);
			temp[i].calculateG();
			temp[i].calculateF();
			// Add it to the list
			result.push_back(temp[i]);
		}
	}

	// Sorting must be done here
	std::sort(result.begin(), result.end());

	return result;

}

vector<Node> Agent::getAdjacentsWithoutParents(Node element){
	vector<Node> result;
	result = map->adjacentHelper(element.getLocation()); // Use helper function to make life easier

	//calculate heuristic of every element
	for (unsigned int i = 0; i < result.size(); i++){
		result[i].setH(0);
		//Set parent just to calculate	G
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
		result[i].clearParent();
	}

	// Sorting must be done here
	std::sort(result.begin(), result.end());

	return result;

}

//Basically remove all the reserved adjacent nodes
// TODO: Check if the values given are correct
std::vector<Node> Agent::getTimedAdjacents(Node element, int res_time, Location ending){
	//First, get the adjacents of the node
	std::vector<Node> temp = map->adjacentHelper(element.getLocation());
	bool reservedElement = false;
	//Then, check if they are reserved at the given time
	std::vector<Node> result;
	for (unsigned int i = 0; i < temp.size(); i++){
		// If it is not reserved, let it be an adjacent
		if (!map->isReserved(temp[i], res_time, id)){
			result.push_back(temp[i]);
		}
		else reservedElement = true;
	}

	//Now that we have all the available nodes, we calculate their real heuristic value
	for (unsigned int i = 0; i < result.size(); i++){
		
		calculateRealHeuristic(&result[i], ending);
		//calculate the rest
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}
	//If a reserved element was found, give the option of stay on your place (Only if it is available)
	if (reservedElement){
		if (!map->isReserved(element, res_time, id)){
			Node repeatedNode(0, 1, element.getX(), element.getY());
			calculateRealHeuristic(&repeatedNode, ending);
			repeatedNode.setParent(element);
			repeatedNode.calculateG();
			repeatedNode.calculateF();
			result.push_back(repeatedNode);
		}
	}

	// Sorting must be done here
	std::sort(result.begin(), result.end());

	return result;
}


vector<Node> Agent::getTimedAdjacentsWithoutParents(Node location, int time){
	//First, get the adjacents of the node
	std::vector<Node> temp = map->adjacentHelper(location.getLocation());
	bool reservedElement = false;
	//Then, check if they where reserved
	std::vector<Node> result;
	for (unsigned int i = 0; i < temp.size(); i++){
		if (!map->isReserved(temp[i], time, id)){
			result.push_back(temp[i]);
		}
		else reservedElement = true;
	}

	//Now that we have all the available nodes, we calculate their real heuristic value
	for (unsigned int i = 0; i < result.size(); i++){
		//Set parent only so you can calculate G, then remove it
		result[i].setParent(location);
		result[i].setH(0);
		//calculate the rest
		result[i].calculateG();
		result[i].calculateF();
		result[i].clearParent();
	}

	// Sorting must be done here
	std::sort(result.begin(), result.end());

	return result;
}


void Agent::calculateSIC(){
	SIC = 0; // Reset the SIC
	if (time_route.size() > 0){
		for (unsigned int i = 1; i < time_route.size(); i++){
			SIC += time_route[i].getF();
		}
	}
}

int Agent::getSic(){
	calculateSIC();
	return SIC;
}

void Agent::clearSpatialLists(bool clearSpatialRoute){
	spatial_openList.clear();
	spatial_closedList.clear();
	index_lower_spatial_openList = 0;
	if (clearSpatialRoute) spatial_route.clear();
}

void Agent::clearTimeLists(bool clearTimeRoute){
	time_openList.clear();
	time_closedList.clear();
	index_lower_time_openList = 0;
	if (clearTimeRoute) time_route.clear();
}

void Agent::calculateRoute(){

	time_route.clear();
	clearSpatialLists(true);

	executeSpatialAstar(actualNode.getLocation(), destination.getLocation());

	time_route = spatial_route; // Because the route was saved on the spatial route
	spatial_route.clear();
	
	calculateSIC();
}

void Agent::ReroutePathUsingCBS(){
	clearSpatialLists(true);
	executeSpatialAstar(
		time_route[time_route.size() - 1].getLocation(), 
		destination.getLocation());
	for (unsigned int i = 0; i < spatial_route.size(); i++){
		time_route.push_back(spatial_route[i]);
	}
	spatial_route.clear();
	calculateSIC();
}

void Agent::ModifyRouteOnConstraints(vector<Constraint> constraints, bool reroute_flag, vector<Constraint> CAT){
	this->CAT = CAT;
	// Lets look for an invalid movement
	for (unsigned int i = 0; i < time_route.size(); i++){
		// check each step for an invalid movement
		if (!validMovement(time_route[i].getLocation(), i, constraints)){
			// If we found an invalid movement
			
			vector<Node> temp_path;

			// Lets push to the temp_path the elements before the incident
			for (unsigned int index = 0; index < i; index++){
				temp_path.push_back(time_route[index]);
			}
			// if
			bool replan = false;
			if (reroute_flag){
				replan = true;
			} else if (i > 0){ // If it isnt the first element
				// If the past step is available to wait on this time
				if (validMovement(time_route[i - 1].getLocation(), i, constraints)){

					// Repeat the last step so the agent waits for some other element to use the other cell
					Node toAdd = temp_path[temp_path.size() - 1];
					toAdd.calculateManhattanHeuristic(destination.getLocation());
					toAdd.setG(toAdd.getG() + 1);
					toAdd.calculateF();
					temp_path.push_back(toAdd);


					//finish adding the elements of the route
					for (unsigned int index = i; index < time_route.size(); index++){
						// Staying has a value of 10, so the values must be recalculated
						Node toAdd = time_route[index];
						toAdd.setG(toAdd.getG() + 1);
						toAdd.calculateF();
						temp_path.push_back(toAdd);
					}
				}
				// if past element is not available, replan
				else replan = true;
			}

			//else if (i == 0){
			//	// If it is the first element+
			//	// Repeat the first step so the agent waits for some other element to use the other cell
			//	Node toAdd = time_route[0];
			//	toAdd.setG(toAdd.getG() + 1);
			//	toAdd.calculateF();
			//	temp_path.push_back(toAdd);


			//	//finish adding the elements of the route
			//	for (unsigned int index = i; index < time_route.size(); index++){
			//		// An extra step was added, so an extra cost should be added
			//		time_route[index].setG(time_route[index].getG() + 1);
			//		time_route[index].calculateF();
			//		temp_path.push_back(time_route[index]);
			//	}

			//}
			else {
				replan = true;
			}

			
			if(replan) { // otherwise recalculate path
				clearSpatialLists(true);
				// If the movement is invalid, we need to replan the path
				// If the constraints wont allow to plan at this timespan, go back one 
				while (!ConstraintAstar(time_route[i - 1].getLocation(), destination.getLocation(), i, constraints)){
					i--;
					temp_path.pop_back();
					clearSpatialLists(true); // to clean (again) the lists
					if (i == 0){
						// Means that a solution couldnt be found, so this node must be descarted
						// First, if this agent has priority, remove it
						priority = false;

						// Now, we must tell that the path wasn't found so this node is not selected
						validSolution = false;

						break;
					}
				}

				 //After a recalculation is done, add the new steps to the path
				if (validSolution){
					int difference = 0;
					if (i > 0) difference = time_route[i - 1].getG();
					for (unsigned int i = 0; i < spatial_route.size(); i++){
						// Update the correct G value
						spatial_route[i].setG(spatial_route[i].getG() + difference);
						// Update the F value
						spatial_route[i].calculateF();
						temp_path.push_back(spatial_route[i]);
					}
				}
				
			}

			//replace the old route with the new one
			time_route = temp_path;
			
		}
	}
				
}

// Move method used when the search is done by CBS
void Agent::moveEntity(unsigned int t){
	
	if (t < time_route.size()){
		actualNode = time_route[t];
	} else if (t > time_route.size()){ // If the time overpasses the amount of steps available
		//Just checking if the acual node is the destination
		if (actualNode == destination){
			active = false;
		}
	}

	cout << "Unit: " << id + 2 << " at location: " << actualNode.getX() << " , " << actualNode.getY();
	if (!active) cout << " This element is finished.";
	cout << endl;
}

// The starting time means the time t where the agent will start moving
void Agent::reserveRoute(int starting_time){
	int reservation_time = starting_time;
	/*
		We are going to reserve d nodes, since after d, we are going to replan our route.
	*/
	bool reserved = false; // Flag that means you need to re-route
	unsigned int reserved_index = 0; // Until what element you need to reroute

	// The limit states how many steps we are going to add to the route
	int limit = 0;
	// If the number of steps is lower than the step limit, then just process the step
	if (steps_limit > time_route.size()) limit = time_route.size();
	// Otherwise only process the step limit moves
	else limit = steps_limit;

	for (int i = 0; i < limit; i++){
		//Check if the node is reserved for this given time
		// Just in case timedAdjacents failed
		if (map->isReserved(time_route[i], reservation_time, id)){
			//If it is reserved
			if (i > 0){
				//Check if the node before that is reserved
				if (map->isReserved(time_route[i - 1], reservation_time, id)){
					//if your last step is reserved for the next time, replan your route
					reserved = true;
					reserved_index = i;
					break;
				}
				else{
					/*
					If the last step is not reserved for this time, then modifly the route and wait
					in the place until the other entity moves.
					*/
					vector<Node> temp_route;

					//Get all the nodes before the incident
					for (int index = 0; index < i; index++){
						temp_route.push_back(time_route[index]);
					}

					//repeat the node so that it can wait on the path, and continue
					temp_route.push_back(time_route[i - 1]);

					//now finish adding the rest of the elements
					for (unsigned int index = i; index < time_route.size(); index++){
						temp_route.push_back(time_route[index]);
					}

					//finally, replace the old route with the new one
					time_route = temp_route;

					//Now, reserve the new element
					map->reserve(reservation_time, time_route[i], id);
					reservation_time++; // increase the time, dont forget this
				}
			}
		}
		else {
			//If it is not reserved, reserve it
			map->reserve(reservation_time, time_route[i], id);
			reservation_time++;
		}
	}

	if (reserved){
		//Eliminate the nodes from the back of the list
		while (time_route.size() > reserved_index){
			time_route.pop_back();
		}

		/*
			Fix: We are now going to replan the route from this point.
			Date: 31/05/2015
			Why?
			Because it is necesary for a more efficient detection of deadlocks.
		*/
		int index_toStart = time_route.size();
		TimeSpaceAstarHelper(
			time_route[time_route.size() - 1].getLocation(), 
			destination.getLocation(), 
			time_route.size());
		// Now from the point you left at reserve your route
		reserveRouteFromIndex(index_toStart);

	}
	else{
		/*
			Now that the route has been replaned, we can now remove all the elements after d
		*/
		while (time_route.size() > steps_limit){
			time_route.pop_back();
		}
	}

}


bool Agent::FindNodeAtList(Node n, vector<Node> list){
	//return std::binary_search(list.begin(), list.end(), n);
	if (std::find(list.begin(), list.end(), n) != list.end()){
		return true;
	}
	return false;
}

bool Agent::FindNodeAtSpatialOpenList(Node n){
	if (std::find(spatial_openList.begin(), spatial_openList.end(), n) != spatial_openList.end()){
		return true;
	}
	return false;
}

bool Agent::FindNodeAtSpatialClosedList(Node n){
	if (std::find(spatial_closedList.begin(), spatial_closedList.end(), n) != spatial_closedList.end()){
		return true;
	}
	return false;
}

bool Agent::FindNodeAtTimeOpenList(Node n){
	//return std::binary_search(list.begin(), list.end(), n);
	if (std::find(time_openList.begin(), time_openList.end(), n) != time_openList.end()){
		return true;
	}
	return false;
}
bool Agent::FindNodeAtTimeClosedList(Node n){
	//return std::binary_search(list.begin(), list.end(), n);
	if (std::find(time_closedList.begin(), time_closedList.end(), n) != time_closedList.end()){
		return true;
	}
	return false;
}



bool Agent::isOnMyRoute(Node n){
	return FindNodeAtList(n, time_route);
}

void Agent::AddNodeToPathAtTimeT(Node n, unsigned int t){
	if (time_route.empty()){ // if the route is empty
		time_route.push_back(n);
	} else if (t < time_route.size()){
		vector<Node> result;
		for (unsigned int i = 0; i < t; i++){
			result.push_back(time_route[i]);
		}

		result.push_back(n);

		for (unsigned int i = t; i < time_route.size(); i++){
			result.push_back(time_route[i]);
		}

		time_route = result;
	}
	else 
		cout << "Insertion into the route unsuccesfull, stopping" << endl;
}

void Agent::ReroutePathUsingSpatialAstar(int time){
	//Erase all the elements after the time stated
	time_route.erase(time_route.begin() + (time + 2), time_route.end());
	//Run astar until the destination from the new spot
	time_closedList.clear();
	time_openList.clear();
	clearSpatialLists(true);
	TimeSpaceAstarHelper(time_route[time - 1].getLocation(), 
		destination.getLocation(), time);
}

// This method modifies the path so that an element outside of the others path is found
void Agent::modifyMap(vector <Node> otherPath){
	map->cleanMap();
	for (unsigned int i = 0; i < otherPath.size(); i++){
		map->setElement(otherPath[i].getX(), otherPath[i].getY(), 99); // Just a value
	}
}


/*
	Returns a simple escape node, meaning, just the cheapest
	adjacent node to the location given in the parameters.
*/
Node Agent::GetSimpleEscapeNode(Location start){

	// A is the node from the location
	Node A(0, start);
	A.setG(0);
	A.setH(0);
	A.calculateF();
	
	Node result;

	// Get the adjacents	
	vector<Node> adjacents = getAdjacents(A, destination.getLocation());
	
	
	/*
		Here, we get the biggest element of the list. Why the biggest?
		Because the buggest means it is the furthest away from the destination.
		Right? RIIGHT?????
	*/
	result = adjacents[adjacents.size() - 1];
	result.clearParent();

	return result;
	
}

// Gets an escape route that is not part of the path indicated in the parameters
Node Agent::GetEscapeNodeNotOnRoute(Location start, vector<Node> path, bool lowerThan){
	// Clear the lists for a better execution of the algorithm
	spatial_openList.clear();
	spatial_closedList.clear();

	
	
	bool nodeFound = false;
	//Let A be the starting point
	Node A(0, start);
	// Assign f, g and h values to A
	A.setG(0);
	A.setH(0);
	A.calculateF();

	//Add A to the open list, At this point, A shoul be the only node on the open list
	spatial_openList.push_back(A);
	index_lower_spatial_openList = 0;

	Node P;
	//If the goal was found, break
	while (!nodeFound){

		// If the open list is empty, no path was found, break
		if (spatial_openList.size() == 0) break;

		//let p be the best node in the open list
		/*	Since the openList is sorted every time an item is added, then the best
		option to select is the first item
		*/
		P = spatial_openList[index_lower_spatial_openList];
		spatial_closedList.push_back(P);
		UpdateSpatialOpenList();

		// Get Adjacents without parent :D
		vector<Node> adjacents = getAdjacentsWithoutParents(P);
		

		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (!FindNodeAtList(adjacents[i], path)){ // If the node is not part of the others route.
				bool isOpposite = false;
				if (lowerThan){
					if (adjacents[i].getLocation() > start)
						isOpposite = true;
				}
				else {
					if (adjacents[i].getLocation() < start)
						isOpposite = true;
				}

				if (isOpposite){
					P = adjacents[i]; // Set it to P
					nodeFound = true; // And break the loop
					break;
				}
			}
			else addToSpatialOpenList(adjacents[i]);
		}
		
		//std::sort(spatial_openList.begin(), spatial_openList.end());
	}

	//Once the element has been found, return it
	return P;
}

void Agent::addToSpatialOpenList(Node n){
	if (!FindNodeAtSpatialClosedList(n)){
		//If it is at the open list
		if (!FindNodeAtSpatialOpenList(n)){
			if (spatial_openList.size() > 0){
				if (n < spatial_openList[index_lower_spatial_openList])
					index_lower_spatial_openList = spatial_openList.size();
			}
			spatial_openList.push_back(n);
		}
		else {
			int index = GetIndexOfElementAtSpatialOpenList(n.getLocation());
			/* 
				If the element on the list has a bigger G value than the
				one we are trying to add, then we should change paths.
			*/
			if (spatial_openList[index].getG() > n.getG()){
				spatial_openList.erase(spatial_openList.begin() + index);
				spatial_openList.push_back(n);
				UpdateIndexSmallerSpatial();
			}
		}
	}
	else {
		//If this element is already in the closed_list, check if it is a better option
		int index = GetIndexOfElementAtSpatialClosedList(n.getLocation());
		if (spatial_closedList[index].getG() > n.getG()){
			spatial_closedList[index] = n;
		}
	}
}

void Agent::addToTimedSpatialOpenList(Node n){

	 // if it isnt at the closed list
		if (!FindNodeAtSpatialOpenList(n)){ // If it is not on the open list
			if (spatial_openList.size() > 0){ // if the list already has elements
				if (n < spatial_openList[index_lower_spatial_openList])// if this element is smaller then the smallest element
					index_lower_spatial_openList = spatial_openList.size(); // update the index
			}
			spatial_openList.push_back(n); // Finally, add the element to the list
		}
		else {// if the element is already on the open list
			int index = GetIndexOfElementAtSpatialOpenList(n.getLocation()); // get its index
			// if the element on the list has a bigger value, then update the values
			if (spatial_openList[index].getG() > n.getG()){
				spatial_openList.erase(spatial_openList.begin() + index);
				spatial_openList.push_back(n);
				UpdateIndexSmallerSpatial();

			}
		}
}

void Agent::AddToTimedSpatialOpenListCAT(Node n){
	// if it isnt at the closed list
	if (!FindNodeAtSpatialOpenList(n)){ // If it is not on the open list
		spatial_openList.push_back(n); // Finally, add the element to the list
	}
	else {// if the element is already on the open list
		int index = GetIndexOfElementAtSpatialOpenList(n.getLocation()); // get its index
		// if the element on the list has a bigger value, then update the values
		if (spatial_openList[index].getG() > n.getG()){
			spatial_openList.erase(spatial_openList.begin() + index);
			spatial_openList.push_back(n);
		}
	}

	UpdateIndexSmallerSpatialCAT();
}

void Agent::addToTimeOpenList(Node n){
	bool atClosedList = FindNodeAtTimeClosedList(n);// check if it is in the closed list
	if (atClosedList){// if it is at the closed list
		//Get the index of the element found
		int index = GetIndexOfElementAtTimeClosedList(n.getLocation());
		/*
			The only way to repeat an element in the closed list is, add it only
			if you are a repeated step, otherwise dont add it.
		*/
		if (n.hasParent()){
			if (n == n.getParent()){
				atClosedList = false;
			}

		}
		
	}

	if (!atClosedList){ // if it isnt at the closed list
		if (!FindNodeAtTimeOpenList(n)){ // If it is not on the open list
			if (time_openList.size() > 0){ // if the list already has elements
				if (n < time_openList[index_lower_time_openList])// if this element is smaller then the smallest element
					index_lower_time_openList = time_openList.size(); // update the index
			}
			time_openList.push_back(n); // Finally, add the element to the list
		}
		else {// if the element is already on the open list
			int index = GetIndexOfElementAtTimeOpenList(n.getLocation()); // get its index
			// if the element on the list has a bigger value, then update the values
			if (time_openList[index].getG() > n.getG()){
				time_openList.erase(time_openList.begin() + index);
				time_openList.push_back(n);
				UpdateIndexSmallerTime();

			}
		}
	}	
}

// Returns the index of an element on a list
int Agent::GetIndexOfElement(vector<Node> list, Node element){
	int result = -1;
	for (unsigned int i = 0; i < list.size(); i++){
		if (element == list[i]){
			result = i;
			break;
		}
	}

	return result;
}

int Agent::GetIndexOfElementAtSpatialOpenList(Location element){
	int result = -1;
	for (unsigned int i = 0; i < spatial_openList.size(); i++){
		if (element == spatial_openList[i].getLocation()){
			result = i;
			break;
		}
	}
	return result;
}

int Agent::GetIndexOfElementAtSpatialClosedList(Location element){
	int result = -1;
	for (unsigned int i = 0; i < spatial_closedList.size(); i++){
		if (element == spatial_closedList[i].getLocation()){
			result = i;
			break;
		}
	}
	return result;
}

int Agent::GetIndexOfElementAtTimeOpenList(Location element){
	int result = -1;
	for (unsigned int i = 0; i < time_openList.size(); i++){
		if (element == time_openList[i].getLocation()){
			result = i;
			break;
		}
	}
	return result;
}

int Agent::GetIndexOfElementAtTimeClosedList(Location element){
	int result = -1;
	for (unsigned int i = 0; i < time_closedList.size(); i++){
		if (element == time_closedList[i].getLocation()){
			result = i;
			break;
		}
	}
	return result;
}


Node Agent::EscapeAstar(Location start){
	int starting_time = time_route.size();
	bool pathFound = false;
	//Let A be the starting point
	Node A(0, start);
	// Assign f, g and h values to A
	A.setG(0);
	/*
		Fix: When looking for an escape node, the only value that is going to be
		taken into account is G, not H.
		Date: 24/06/2015
		Why?
		Because we don't actually know what is going to be our destination, so there
		is no point in setting an H value.
	*/
	A.setH(0);
	A.calculateF();

	//Add A to the open list, At this point, A shoul be the only node on the open list
	spatial_openList.push_back(A);

	Node P;
	//If the goal was found, break
	while (!pathFound){

		// If the open list is empty, no path was found, break
		if (spatial_openList.size() == 0) break;

		//let p be the best node in the open list
		/*	Since the openList is sorted every time an item is added, then the best
		option to select is the first item
		*/
		
		P = spatial_openList[index_lower_spatial_openList];
		spatial_closedList.push_back(P);
		UpdateSpatialOpenList();


		vector<Node> adjacents = getAdjacentsWithoutParents(P);
		

		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (!map->isReserved(adjacents[i], starting_time, id)){ // If we found an empty element
				P = adjacents[i]; // Set it to P
				pathFound = true; // And break the loop
				break;
			}
			
			else {// Else, proceed with the normal Astar
				addToSpatialOpenList(adjacents[i]);
			}
			
			//std::sort(spatial_openList.begin(), spatial_openList.end());
		}
	}
	//Once the element has been found, return it
	return P;
}

void Agent::MoveToClosestEscapeElement(bool KeepRoute, Location start){
	
	//Clear the lists first
	clearSpatialLists(false);
	//First, lets get the closest escape Element
	Node escapeNode = EscapeAstar(start);

	if (!KeepRoute){
		// If you want to clear the route, clear it and return t to 0
		time_route.clear();
	}
	//Before restarting, we need to prepare the elements for the search
	// First step, clear the lists
	time_openList.clear();
	time_closedList.clear();

	// Since we will use this element twice, a pointer is created
	Node* lastNode = &time_route[time_route.size() - 1];
	// Second step, remove the parent of the last element of the route
	lastNode->clearParent();

	// now we can restart the search, but we look for the escape Node
	TimeSpaceAstarHelper(
		lastNode->getLocation(), 
		escapeNode.getLocation(),
		time_route.size());

	// Now we have the route to the escape route

	lastNode = NULL; // CLEAN UP
}



void Agent::RepeatStepAtIndex(unsigned int index, int times){
	//If the index is in the correct ranfge
	if (index < time_route.size()){
		vector<Node> temp;
		// Add all the elements before the index
		for (unsigned int i = 0; i < index; i++){
			temp.push_back(time_route[i]);
		}

		// Add the index element the number of times necesary
		for (int i = 0; i < times; i++){
			temp.push_back(time_route[index]);
		}

		// Add the index element and the rest of the elements
		for (unsigned int i = index; i < time_route.size(); i++){
			temp.push_back(time_route[i]);
		}

		time_route = temp; // reasign the value

	}
	else cout << "There is an error on the method RepeatStepAtIndex, invalid value" << endl;
}


void Agent::PushElementAtTheBackOfRoute(Node val){
	time_route.push_back(val);
}

/*
	Starts reserving the nodes from the time route starting at the index stated until the 
	end of the route.
*/
void Agent::reserveRouteFromIndex(unsigned int index){
	
	for (unsigned int i = index; i < time_route.size(); i++){
		if (!map->isReserved(time_route[i], i, id)){
			map->reserve(i, time_route[i], id);
		}
		else {
			//Just to test if timed adjacents is failing
			cout << "Breaking reserve route from index" << endl;
		}
	}
}

void Agent::ActivateReplanFlag(){
	stepsTaken = steps_limit;
}

void Agent::UpdateSpatialOpenList(){
	spatial_openList.erase(spatial_openList.begin() + index_lower_spatial_openList);
	UpdateIndexSmallerSpatial();
}

void Agent::UpdateTimeOpenList(){
	time_openList.erase(time_openList.begin() + index_lower_time_openList);
	UpdateIndexSmallerTime();
}

void Agent::UpdateSpatialOpenListCAT(){
	spatial_openList.erase(spatial_openList.begin() + index_lower_spatial_openList);
	UpdateIndexSmallerSpatialCAT();
}



void Agent::UpdateIndexSmallerSpatial(){
	index_lower_spatial_openList = 0;
	if (spatial_openList.size() > 1){
		for (unsigned int i = 1; i < spatial_openList.size(); i++){
			if (spatial_openList[i] < spatial_openList[index_lower_spatial_openList]){
				index_lower_spatial_openList = i;
			}
		}
	}
	 
}

void Agent::UpdateIndexSmallerTime(){
	index_lower_time_openList = 0;
	if (time_openList.size() > 1){
		for (unsigned int i = 0; i < time_openList.size(); i++){
			if (time_openList[i] < time_openList[index_lower_time_openList]){
				index_lower_time_openList = i;
			}
		}
	}
	
}

/*
	Checks if there are two nodes with same F values. If so, it checks which node has less occurances on 
	the CAT table. That node is selected as the lower node.
*/
void Agent::UpdateIndexSmallerSpatialCAT(){
	index_lower_spatial_openList = 0;
	if (spatial_openList.size() > 1){
		for (unsigned int i = 1; i < spatial_openList.size(); i++){
			if (spatial_openList[i] < spatial_openList[index_lower_spatial_openList]){
				index_lower_spatial_openList = i;
			}
			else if (spatial_openList[i].getF() == spatial_openList[index_lower_spatial_openList].getF()){
				int ocurrences_i = FindNumberOcurrancesCAT(spatial_openList[i].getLocation());
				int ocurrences_actual = FindNumberOcurrancesCAT(spatial_openList[index_lower_spatial_openList].getLocation());
				if (ocurrences_i < ocurrences_actual){
					index_lower_spatial_openList = i;
				}
			}
		}
	}
}

/*
	Checks if a movement is valid against the constraint list
	Fix: when a constraint is on the list of constraints, it means that the element with that id,
	cant occupy that vertex at the time t.
	Date: 09/070/2015
	Why?
	Misunderstanding of CBS by me, SO SORRY :(
*/
bool Agent::validMovement(Location location, int time, vector<Constraint> constraints){
	for (unsigned int i = 0; i < constraints.size(); i++){
		if (constraints[i].t != 0){ // It is ridiculous to have a constraint on time 0 ...
			if (constraints[i].id == id){
				if (constraints[i].t == time){
					if (location == constraints[i].location){
						return false;
					}
				}
			}
		}
	}

	return true;
}

void Agent::SanitizePath(){
	// If the last element is in fact, the destination
	if (time_route[time_route.size() - 1] == destination){
		int counter = 0; // counter to see how many repetitions we have
		for (int i = time_route.size() - 1; i >= 0; i--){
			// If the element is the destination, count
			if (time_route[i] == destination) counter++;
			// else, escape the for loop
			else break;
		}
		// If we have more than 1 repetition
		while (counter > 1){
			// remove the last repetition
			time_route.pop_back();
			// decrease counter
			counter--;
		}
	}
}

void Agent::resetElement(){
	actualNode = startingPoint;
	clearSpatialLists(true);
	clearTimeLists(true);
	validSolution = false;
}

int Agent::getManhattanBetweenNodes(){
	return abs(startingPoint.getLocation().x - destination.getLocation().x) + 
		abs(startingPoint.getLocation().y - destination.getLocation().y);
}

bool Agent::FindSpecialCaseCBS(Location location, int t, vector<Constraint> constraints){
	bool result = false;
	for (unsigned int i = 0; i < constraints.size(); i++){
		if (constraints[i].t == t + 1){
			if (constraints[i].location == location){
				if (constraints[i].id == id){
					vector<Node> adjacents = getAdjacentsWithoutParents(Node(0, location));
					result = true;
					// TODO: Uncomment if needed
					/*for (unsigned int i = 0; i < adjacents.size(); i++){
						result = result && !validMovement(adjacents[i].getLocation(), t + 1, constraints);
					}*/
					break;
				}
			}
		}
	}

	return result;
}

int Agent::FindNumberOcurrancesCAT(Location location) {
	int result = 0;
	for (unsigned int i = 0; i < CAT.size(); i++){
		if (CAT[i].location == location) result++;
	}

	return result;
}