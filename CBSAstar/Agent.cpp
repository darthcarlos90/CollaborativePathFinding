#include "Agent.h"


Agent::Agent(Node location,Node destination, Map* m, int id, unsigned int D):
actualNode(location)
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
	has_partial_destination = false;
}

Agent::~Agent(void){
	map = NULL;//There is only one map, it is not my job to delete it
}

/*
	At the copy constructor, I am only going to add the stuff that I need to use when
	passing agents as parameters in methods, that way I hope to save some innecesary coding
	or passing some innecesary elements;
*/
//Agent::Agent(const Agent& other){
//	// For now, I just need the path and the id perhaps
//	this->id = other.id;
//	this->time_route = other.time_route;
//	this->actualNode = other.actualNode;
//	this->destination = other.destination;
//
//	//Things that I dont need
//	map = NULL;
//
//	// TODO: If I need more elements at the copy constructor, just add them
//}


/*
Astar algorithm built from the Tutorial 2 on AI and from the
video at this link: https ://www.youtube.com/watch?v=KNXfSOx4eEE

*/
void Agent::executeSpatialAstar(Node start, Node finish){
	// Firstly cleaer the routes
	spatial_openList.clear();
	spatial_closedList.clear();

	bool pathFound = false;
	//Let A be the starting point
	Node A = start;
	// Assign f, g and h values to A
	A.setG(0);
	A.calculateManhattanHeuristic(finish);
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
		P = spatial_openList[0];
		spatial_closedList.push_back(P);
		spatial_openList.erase(spatial_openList.begin());

		/*
			Fix:The comparison here was changed, instead to see if the type is of 3, we compare against
			the final node.
			Date: 01/05/2015
			Why?
			Because originally it was comparing against 3, but because we are on time-space Astar, that
			methodology is used no more, so now we compare to see if it is the destination node.
		*/
		if (P == finish){
			pathFound = true;
			break;
		}

		vector<Node> adjacents = getAdjacents(P, finish);
		for (unsigned int i = 0; i < adjacents.size(); i++){
			addToSpatialOpenList(adjacents[i]);
		}
		std::sort(spatial_openList.begin(), spatial_openList.end());
	}

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
		spatial_route.push_back(inverse_route[i]);
	}
}

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
		Node pastNode = actualNode;// for debug
		actualNode = time_route[t - 1];
		stepsTaken++;
		map->setElement(pastNode.getX(), pastNode.getY(), 0); // For debugging
	}
		
	map->setElement(actualNode.getX(), actualNode.getY(), id + 2);
	

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
		TimeSpaceAstarHelper(time_route[time_route.size() - 1], // The las node on the path
			destination, // The destination of the pathfinding
			new_starting_time); // So that the reroute starts at the end of the route
		reserveRouteFromIndex(new_starting_time); // The index will be the last index of the table
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
		Steps 1 and 2 are described in the function that will execute.
	*/
	TimeSpaceAstarHelper(actualNode, destination, starting_time);
	
	// Now that we have a route, reserve it
	reserveRoute(starting_time + 1);
}

/*
	This method is create to execute Time-Space Astar starting from the last 
	element on the time route, till it finds the end (or reaches the limit of
	steps).
*/
void Agent::executeTimeSpaceAstarFromLastIndex(){
	int index = time_route.size();
	TimeSpaceAstarHelper(time_route[time_route.size() - 1], destination, time_route.size());
	reserveRouteFromIndex(index);
}

void Agent::TimeSpaceAstarHelper(Node start, Node finish, int time){	
	// In case they're not empty
	time_closedList.clear();
	time_openList.clear();
	/*
	Step 1: Calculate the route.
	- Calculate the route using regular Astar, BUT the H used on every node will be
	the G value calculated in the reverse Astar.
	- When calculating the route, check if that element has been reserved by someone
	for some time t, otherwise, it will not be used.
	*/
	bool pathFound = false;
	Node A = start; //Let A be the starting point

	A.setG(10);
	calculateRealHeuristic(&A, finish);
	A.calculateF();
	/*
		Fix: We need to also reserve the starting node of each of the agents.
		Date: 01/06/2015
		Why?
		Because this could lead to other elements illegally moving to the starting point
		in a given time.
	*/
	map->reserve(time, start, id);
	
	time_openList.push_back(A); //We put it on the open list
	Node P;
	
	

	int counter = steps_limit; //This counter will help us find a partial path
	
	//if the open list is empty, or the goal was found, quit
	while (!pathFound){
		if (time_openList.empty()) break; // If the open list is empty, no path found
		//let p be the best node in the open list
		/*	Since the openList is sorted every time an item is added, then the best
			option to select is the first item
		*/
		P = time_openList[0];
		time_closedList.push_back(P);
		time_openList.erase(time_openList.begin()); 

		// If P is the goal node, then finished this 
		if (P == finish){
			pathFound = true;
			break;
		}

		//otherwise
		// Get the adjacents of node P
		vector<Node> adjacents = getTimedAdjacents(P, time);

		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (adjacents[i] == P){ //if the option is to stay, dont ignore it
				time_openList.push_back(adjacents[i]);
			}
			else {
				if (!FindNodeAtList(adjacents[i], time_closedList)){
					if (!FindNodeAtList(adjacents[i], time_openList))
						time_openList.push_back(adjacents[i]);
				}
			}
		}
		std::sort(time_openList.begin(), time_openList.end());
		time++; //Increase time
		counter--; // Decrease the counter
		if (counter == 0){ //If the counter reached 0
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
			//Reset the counter in case no partial paths where found
			//counter = d/2; //but make it smaller
		}
	}

	/* Step 2: Once the route has been found, reserve your path.
		- Only reserve the blocks of your path where nPath <= steps
	*/
		
	//If we have a partial path, populate the route with the path
	if (partial_path_nodes.size() > 0){
		//If we have more than 1 partial path, lets decide
		if (partial_path_nodes.size() > 1){
			int smaller_index =  0;
			int smaller_value = 0;
			//Get the element with the smallest value from d + 1 till the end
			for (unsigned int i = 0; i < partial_path_nodes.size(); i++){
				vector<Node> around = getTimedAdjacents(partial_path_nodes[i], time); // get the adjacents
				std::sort(around.begin(), around.end()); //sort so the first element is the smaller
				spatial_route.clear(); // clear the route
					
				executeSpatialAstar(around[0], destination);
				int route_value = 0;
					
				//Get the value of the route
				for (unsigned int j = 0; j < spatial_route.size(); j++){
					route_value += spatial_route[j].getF();
				}
				
				//see which route has the smaller value
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

	//First populate the path
	vector<Node> temp;

	if (P.hasParent()){
		while (P.hasParent()){
			temp.push_back(P);
			P = P.getParent();
		}
	}
	else temp.push_back(P);

	//get the nodes in order
	for (int i = temp.size() - 1; i >= 0; i--){
		time_route.push_back(temp[i]);
	}
	// TODO: Updated for now
	//t = time; //update the value of t	
}


void Agent::executeSpatialAstarUntilFound(Node start, Node toFind){
	if (!FindNodeAtList(toFind, spatial_closedList)){
		executeSpatialAstar(start, toFind);
	}
}

void Agent::calculateRealHeuristic(Node* toCalculate, Node finish){
	executeSpatialAstarUntilFound(finish, *toCalculate);
	for (unsigned int i = 0; i < spatial_closedList.size(); i++){
		if (spatial_closedList[i] == *toCalculate){
			toCalculate->setH(spatial_closedList[i].getG());
			break;
		}
	}

}


vector<Node> Agent::getAdjacents(Node element, Node ending){
	vector<Node> result;

	result = map->adjacentHelper(element); // Use helper function to make life easier

	//calculate heuristic of every element
	for (unsigned int i = 0; i < result.size(); i++){
		result[i].calculateManhattanHeuristic(ending);
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}

	return result;
}


vector<Node> Agent::getAdjacents2(Location location,int time){
	Node n(0,0, location.x, location.y);
	return getTimedAdjacents(n, time);
}

//Basically remove all the reserved adjacent nodes
std::vector<Node> Agent::getTimedAdjacents(Node element, int res_time){
	//First, get the adjacents of the node
	std::vector<Node> temp = map->adjacentHelper(element);
	bool reservedElement = false;
	//Then, check if they where reserved
	std::vector<Node> result;
	for (unsigned int i = 0; i < temp.size(); i++){
		if (!map->isReserved(temp[i], res_time, id)){
			result.push_back(temp[i]);
		}
		else reservedElement = true;
	}

	//Now that we have all the available nodes, we calculate their real heuristic value
	for (unsigned int i = 0; i < result.size(); i++){
		
		calculateRealHeuristic(&result[i], destination);
		//calculate the rest
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}
	//If a reserved element was found, give the option of stay on your place (Only if it is available)
	if (reservedElement){
		if (!map->isReserved(element, res_time, id)){
			element.setG(10);
			calculateRealHeuristic(&element, destination);
			element.calculateF();
			result.push_back(element);
		}
	}

	return result;
}



//TODO: Commented to see if this will be needed later
//void Agent::setTime(int time_to_set){
//	t = time_to_set;
//
//	//Why dis??
//	/*
//		This was executed so that after the time that was passed, the element
//		stayed in the same place in the route.
//	*/
//	for (int i = 0; i < t; i++){
//		time_route.push_back(actualNode);
//	}
//}



void Agent::calculateSIC(){
	if (time_route.size() > 0){
		for (unsigned int i = 0; i < time_route.size(); i++){
			SIC += time_route[i].getF();
		}
	}
}

int Agent::getSic(){
	if (SIC == 0) calculateSIC();
	return SIC;
}

void Agent::calculateRoute(){
	time_route.clear();
	spatial_route.clear();
	// If there is a partial destination
	if (has_partial_destination){
		//First calculate the route to the partial destination
		executeSpatialAstar(actualNode, partialDestination);
		
		//Now go the the actual destination
		executeSpatialAstar(partialDestination, destination);
	} else executeSpatialAstar(actualNode, destination);
	
	time_route = spatial_route; // Because the route was saved on the spatial route
	spatial_route.clear();
	calculateSIC();
}

void Agent::ReroutePathUsingCBS(){
	spatial_route.clear();
	spatial_openList.clear();
	spatial_closedList.clear();
	executeSpatialAstar(time_route[time_route.size() - 1], destination);
	for (unsigned int i = 0; i < spatial_route.size(); i++){
		time_route.push_back(spatial_route[i]);
	}
	spatial_route.clear();
	calculateSIC();
}

void Agent::ModifyRouteOnConstraints(vector<Constraint> constraints){
	
	for (unsigned int i = 0; i < constraints.size(); i++){
		Constraint c = constraints[i];
		//If the constraint is from another agent
		if (c.id != id){
			// If this agent has a movement at time t
			if (time_route.size() > c.t){
				//If the movement at time t is the same as the constraint
				if (time_route[c.t].getLocation() == c.location){
					//Make the element wait for a t = 1 before moving to that spot
					vector<Node> temp_path;
					// push all the elements of the path from 0 till t -1 of the constraint
					if (c.t == 0){
						/*
							If the conflict occurs on the first movement of both elements,
							then the other element must wait on its first step before moving
							to some other step.
						*/
						temp_path.push_back(actualNode);
					}
					else {
						for (unsigned int i = 0; i < c.t; i++){
							temp_path.push_back(time_route[i]);
						}
					}
					// Repeat the last step so the agent waits for some other element to use the other cell
					temp_path.push_back(temp_path[temp_path.size() - 1]);

					//finish adding the elements of the route
					for (unsigned int i = c.t; i < time_route.size(); i++){
						temp_path.push_back(time_route[i]);
					}

					//replace the old route with the new one
					time_route = temp_path;
				}
			}
		}
	}
	
}

void Agent::moveEntity(unsigned int t){
	
	if (t < time_route.size()){
		actualNode = time_route[t];
		cout << "Unit: " << id + 2 << " at location: " << actualNode.getX() << " , " << actualNode.getY() << endl;
	} else if (t > time_route.size()){ // If the time overpasses the amount of steps available
		//Just checking if the acual node is the destination
		if (actualNode == destination){
			active = false;
		}
	}
}

// The starting time means the time t where the agent will start moving
void Agent::reserveRoute(int starting_time){
	int reservation_time = starting_time;
	/*
		We are going to reserve d nodes, since after d, we are going to replan our route.
	*/
	bool reserved = false; // Flag that means you need to rerote
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
					for (unsigned int index = 0; index < i; index++){
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
		TimeSpaceAstarHelper(time_route[time_route.size() - 1], destination, time_route.size());
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

bool Agent::isOnMyRoute(Node n){
	return FindNodeAtList(n, time_route);
}

void Agent::AddNodeToPathAtTimeT(Node n, unsigned int t){
	if (time_route.empty()){ // if the route is empty
		time_route.push_back(n);
	} else if (t < time_route.size()){
		vector<Node> result;
		for (unsigned int i = 0; i <= t; i++){
			result.push_back(time_route[i]);
		}

		result.push_back(n);

		for (unsigned int i = t + 1; i < time_route.size(); i++){
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
	spatial_closedList.clear();
	spatial_openList.clear();
	spatial_route.clear();
	time_route[time - 1].clearParent(); // Because we are starting from this point
	TimeSpaceAstarHelper(time_route[time - 1], destination, time);
}

// This method modifies the path so that an element outside of the others path is found
void Agent::modifyMap(vector <Node> otherPath){
	map->cleanMap();
	for (unsigned int i = 0; i < otherPath.size(); i++){
		map->setElement(otherPath[i].getX(), otherPath[i].getY(), 99); // Just a value
	}
}
Node Agent::GetEscapeNodeNotOnRoute(Node start, vector<Node> path, bool lowerThan){
	// Clear the lists for a better execution of the algorithm
	spatial_openList.clear();
	spatial_closedList.clear();

	
	
	bool nodeFound = false;
	//Let A be the starting point
	Node A = start;
	// Assign f, g and h values to A
	A.setG(0);
	A.calculateManhattanHeuristic(destination);
	A.calculateF();

	//Add A to the open list, At this point, A shoul be the only node on the open list
	spatial_openList.push_back(A);

	Node P;
	//If the goal was found, break
	while (!nodeFound){

		// If the open list is empty, no path was found, break
		if (spatial_openList.size() == 0) break;

		//let p be the best node in the open list
		/*	Since the openList is sorted every time an item is added, then the best
		option to select is the first item
		*/
		P = spatial_openList[0];
		spatial_closedList.push_back(P);
		spatial_openList.erase(spatial_openList.begin());


		vector<Node> adjacents = getAdjacents(P, destination);

		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (!FindNodeAtList(adjacents[i], path)){ // If the node is not part of the others route.
				bool isOpposite = false;
				if (lowerThan){
					if (adjacents[i].getLocation() > start.getLocation())
						isOpposite = true;
				}
				else {
					if (adjacents[i].getLocation() < start.getLocation())
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
		std::sort(spatial_openList.begin(), spatial_openList.end());
	}

	//Once the element has been found, return it
	return P;
}

void Agent::addToSpatialOpenList(Node n){
	if (!FindNodeAtList(n, spatial_closedList)){
		if (!FindNodeAtList(n, spatial_openList))
			spatial_openList.push_back(n);
	}
}


Node Agent::EscapeAstar(Node start){
	int starting_time = time_route.size();
	bool pathFound = false;
	//Let A be the starting point
	Node A = start;
	// Assign f, g and h values to A
	A.setG(0);
	A.calculateManhattanHeuristic(destination);
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
		P = spatial_openList[0];
		spatial_closedList.push_back(P);
		spatial_openList.erase(spatial_openList.begin());


		vector<Node> adjacents = getAdjacents(P, destination);
		// Sort the adjacents to the cheapest one
		std::sort(adjacents.begin(), adjacents.end());

		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (!map->isReserved(adjacents[i], starting_time, id)){ // If we found an empty element
				P = adjacents[i]; // Set it to P
				pathFound = true; // And break the loop
				break;
			}
			
			else {// Else, proceed with the normal Astar
				addToSpatialOpenList(adjacents[i]);
			}
			std::sort(spatial_openList.begin(), spatial_openList.end());
		}
	}

	//Once the element has been found, return it
	return P;
}

void Agent::MoveToClosestEscapeElement(bool KeepRoute, Node start){
	
	//Clear the lists first
	spatial_openList.clear();
	spatial_closedList.clear();
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
	TimeSpaceAstarHelper(*lastNode, escapeNode, time_route.size());

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
	int time = index + 1;
	for (unsigned int i = index; i < time_route.size(); i++){
		if (!map->isReserved(time_route[i], time, id)){
			map->reserve(time, time_route[i], id);
			time++;
		}
		else {
			//Just to test if timed adjacents is failing
			cout << "Breaking" << endl;
		}
	}
}

void Agent::ActivateReplanFlag(){
	stepsTaken = steps_limit;
}

void Agent::setPartialDestination(Node val){
	partialDestination = val;
	has_partial_destination = true;
}