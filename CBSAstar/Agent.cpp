#include "Agent.h"


Agent::Agent(Node location,Node destination, Map* m, int id, int D):
actualNode(location)
{
	map = m;
	active = true;
	reachedD = false;
	this->id = id;
	this->d = D;
	this->destination = destination;
	replan = false;
	tempD = 0;
}

Agent::~Agent(void){
	map = NULL;//There is only one map, it is not my job to delete it
}

/*
Astar algorithm built from the Tutorial 2 on AI and from the
video at this link: https ://www.youtube.com/watch?v=KNXfSOx4eEE

*/
void Agent::executeSpatialAstar(Node start, Node finish){
	bool pathFound = false;
	//Let A be the starting point
	Node A = start;
	// Assign f, g and h values to A
	A.calculateG();
	A.calculateManhattanHeuristic(finish);

	//Add A to the open list, At this point, A shoul be the only node on the open list
	spatial_openList.push_back(A);

	Node P;
	//if the open list is empty, or the goal was found, quit
	while (!pathFound || spatial_openList.size() > 0){

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

		vector<Node> adjacents = getAdjacents(P);
		for (unsigned int i = 0; i < adjacents.size(); i++){
			if (!AtClosedList(adjacents[i])){
				if (!AtOpenList(adjacents[i]))
					spatial_openList.push_back(adjacents[i]);
			}
		}
		std::sort(spatial_openList.begin(), spatial_openList.end());
	}

	//Once the path has been found, retrace your steps
	while (P.hasParent()){
		spatial_route.push_back(P);
		P = P.getParent();
	}

	
	//vector<Node> inverse_route = P.getFamily();
	//for (int i = inverse_route.size() - 1; i >= 0; i--){
	//	spatial_route.push_back(inverse_route[i]);
	//}
	////Print the route, just for debugging
	//for (int i = 0; i < spatial_route.size(); i++){
	//	system("cls");
	//	Node n = spatial_route[i];
	//	map->setElement(n.getX(), n.getY(), 9);
	//	map->printData();
	//}
}

void Agent::move(int time_to_move){
	/*
	Step 3: Every t, advance one spot in the road.
	- If d has been reached or we reaced to the end of the road, stop;
	otherwise, continue advancing.
	*/

	if (stepsTaken < d){
		t++;
		if (time_route.size() == 0) {
			active = false; //we reached the end of the route 
		}
		else {
			actualNode = time_route[0];
			time_route.erase(time_route.begin());
		}
		stepsTaken++;
		map->setElement(actualNode.getX(), actualNode.getY(), id);
		system("cls");
		map->printData();
	}
	else if(stepsTaken == d){
		/*
		Step 4: If the d has been reached:
		- If the destination has been reached:
			- Set this Agent as unactive
		- Repeat from step 1, BUT
		- Now use Heuristic distances (I still can't understand why, but thats what the paper says :S)
		*/
		if (replan){
			d = tempD;
			tempD = 0;
			replan = false;
			active = true;
		}
		else {
			if (time_route.size() == 0) active = false;
			reachedD = true;
			t++;
		}
		time_openList.clear();
		time_closedList.clear();
		time_route.clear(); //clear the route as a new route will start to be calculated
		actualNode.clearParent(); //Clear the parent, so this is a true starting point
		stepsTaken = 0;
		executeTimeSpaceAstar();
		move(time_to_move);
	}
	
}


/*
This algorithm was created based on the work done by David Silver in his paper
named Cooperative Pathfinding. Some other code was also added by myself.
*/
void Agent::executeTimeSpaceAstar(){
	/*
	Steps 1 and 2 are described in the function that will execute.
	*/
	

	TimeSpaceAstarHelper(actualNode, destination);
	
	//TODO: Find a way to make it move automatically
	//The user will tell when to move, for now
	// That is why the move method is commented
	//move();
}

void Agent::TimeSpaceAstarHelper(Node start, Node finish){
	int time = t + 1;
	//TODO: Find the bug, there must be something going wrong here 
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
	map->setElement(A.getX(), A.getY(), id);
	time_openList.push_back(A); //We put it on the open list
	Node P;
	
	
	/*
	(Sorry for the disorder of the steps :( )
	Step 5: If you are in the end, stay there untl someone else needs your space.
	- In case that someone needs to pass, move n places away, then, star again calculating
	the spatial Astar.
	*/
	//Check if you already finished
	if (!active){
		//Set the cost of staying to 0
		A.setG(0);
		//If finished, stay the next seven steps where you are, unless someone needs to pass
		for (int i = 0; i < d; i++){
			//if someone needs to use your space
			if (map->isReserved(A, time)){
				//move out
				//First, get the adjacents of that specific element
				vector<Node> adjacents = getTimedAdjacents(A, time);

				//Next, sort them to get the cheapest one
				std::sort(adjacents.begin(), adjacents.end());

				time_route.push_back(adjacents[0]);//set to the route the closest element
				/*
				Step 6: Recalculate how much you are gonna move and move back to your position.
				- Set active to true, so that the element starts using Astar to go back to its place.
				*/
				active = true; //set to active, so the element can come back to its position after its finished
				break;// exit method;
			}

			//else
			else{
				//add to the route that you are staying in your place that period of time
				time_route.push_back(A);
				time++;
			}
		}
	}
	// If not finished, proceed to do the other normal stuff.
	else {
		//if the open list is empty, or the goal was found, quit
		while (!pathFound){

			//let p be the best node in the open list
			/*	Since the openList is sorted every time an item is added, then the best
			option to select is the first item
			*/
			P = time_openList[0];
			time_closedList.push_back(P);
			time_openList.erase(time_openList.begin()); //Lets keep it here for a while
			//here, we dont erase the element we just used from the open list, this because we want to be able
			//to "wait" on the same place

			// If P is the goal node, then finished this 
			if (P == finish){
				pathFound = true;
				break;
			}

			//otherwise
			vector<Node> adjacents = getTimedAdjacents(P, time);

			for (unsigned int i = 0; i < adjacents.size(); i++){
				if (!AtTimedClosedList(adjacents[i])){
					if (!AtTimedOpenList(adjacents[i])) 
						time_openList.push_back(adjacents[i]);
				}
			}
			std::sort(time_openList.begin(), time_openList.end());
			time++; //Increase time
		}

		/*
		Step 2: Once the route has been found, reserve theon your path.
		- Only reserve the blocks of your path where nPath <= steps
		*/

		//First populate the path
		vector<Node> temp;
		//while (P != A){
		
		while (P.hasParent()){
			temp.push_back(P);
			P = P.getParent();
		}

		//get the nodes in order
		for (int i = temp.size() - 1; i >= 0; i--){
			time_route.push_back(temp[i]);
		}

		if (time_route.size() < d){
			Node n = time_route[time_route.size() - 1];
			while (time_route.size() < d){
				time_route.push_back(n);
			}
		}

		int reservation_time = t + 1;
		//reserve d nodes
		bool reserved = false;
		int reserved_index = 0;
		for (int i = 0; i < d; i++){
			//Check if the node is reserved for this given time
			if (map->isReserved(time_route[i], reservation_time)){
				reserved = true;
				reserved_index = i;
				break;
			}
			else {
				map->reserve(reservation_time, time_route[i], id);
				reservation_time++;
			}
		}

		if (reserved){
			//Eliminate the nodes from the back of the list
			while (time_route.size() > reserved_index){
				time_route.pop_back();
			}

			//Replan the route recursively using the last node
			/*t = reservation_time;

			TimeSpaceAstarHelper(time_route[time_route.size() - 1], finish);*/
			replan = true;
			tempD = d;
			d = time_route.size();
			
		}
	}

	
}


void Agent::executeSpatialAstarUntilFound(Node start, Node toFind){
	if (!AtClosedList(toFind)){
		executeSpatialAstar(start, toFind);
	}
}

bool Agent::AtClosedList(Node n){
	/*bool found = false;
	if (spatial_closedList.size() > 0){
		for (unsigned int i = 0; i < spatial_closedList.size(); i++){
			if (spatial_closedList[i] == n){
				found = true;
				break;
			}
		}
	}

	return found;*/

	//return ListContains(spatial_closedList, n);

	if (!spatial_closedList.empty()){
		if (std::find(spatial_closedList.begin(), spatial_closedList.end(), n) != spatial_closedList.end()){
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

void Agent::calculateRealHeuristic(Node* toCalculate, Node finish){
	executeSpatialAstarUntilFound(finish, *toCalculate);
	/*for (unsigned int i = 0; i < spatial_closedList.size(); i++){
		if (spatial_closedList[i] == *toCalculate){
			toCalculate->setH(spatial_closedList[i].getG());
			break;
		}
	}*/

	toCalculate->setH(spatial_closedList[spatial_closedList.size() - 1].getG());

	//TODO: Make this faster, using the last index of the list, since it was the last element to be added
}


vector<Node> Agent::getAdjacents(Node element){
	vector<Node> result;

	result = map->adjacentHelper(element); // Use helper function to make life easier

	//calculate heuristic of every element
	for (unsigned int i = 0; i < result.size(); i++){
		result[i].calculateManhattanHeuristic(destination);
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}

	return result;
}

//Basically remove all the reserved adjacent nodes
std::vector<Node> Agent::getTimedAdjacents(Node element, int res_time){
	//First, get the adjacents of the node
	std::vector<Node> temp = map->adjacentHelper(element);

	//Then, check if they where reserved
	std::vector<Node> result;
	for (unsigned int i = 0; i < temp.size(); i++){
		if (!map->isReserved(temp[i], res_time)){
			result.push_back(temp[i]);
		}
	}

	//Now that we have all the available nodes, we calculate their real heuristic value
	for (unsigned int i = 0; i < result.size(); i++){
		//If we reached the D, use manhattan, else, use the real heuristic
		if (reachedD) result[i].calculateManhattanHeuristic(destination);
		else calculateRealHeuristic(&result[i], destination);
		//calculate the rest
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}

	return result;
}

bool Agent::AtTimedClosedList(Node n){
	/*bool found = false;
	if (time_closedList.size() > 0){
		for (unsigned int i = 0; i < time_closedList.size(); i++){
			if (time_closedList[i] == n){
				found = true;
				break;
			}
		}
	}
	return found;*/

	//return ListContains(time_closedList, n);

	if (!time_closedList.empty()){
		if (std::find(time_closedList.begin(), time_closedList.end(), n) != time_closedList.end()){
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
	
}


bool Agent::AtTimedOpenList(Node n){
	/*bool found = false;
	for (int i = 0; i < time_openList.size(); i++){
		if (time_openList[i] == n){
			found = true;
			break;
		}
	}

	return found;*/
	if (!time_openList.empty()){
		if (std::find(time_openList.begin(), time_openList.end(), n) != time_openList.end()){
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}

	//return ListContains(time_openList, n);
}

void Agent::setTime(int time_to_set){
	t = time_to_set;

	for (int i = 0; i < t; i++){
		time_route.push_back(actualNode);
	}
}

bool Agent::AtOpenList(Node n){
	if (!spatial_openList.empty()){
		if (std::find(spatial_openList.begin(), spatial_openList.end(), n) != spatial_openList.end()){
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}