#include "Agent.h"


Agent::Agent(Node location,Node destination, Map* m, int id, unsigned int D):
actualNode(location)
{
	map = m;
	active = true;
	this->id = id;
	this->d = D;
	this->destination = destination;
	replan = false;
	tempD = 0;
	SIC = 0;
	map->setElement(actualNode.getX(), actualNode.getY(), id + 2);
	map->setElement(destination.getX(), destination.getY(), id + 2);
	system("cls");
	map->printData();
}

Agent::~Agent(void){
	map = NULL;//There is only one map, it is not my job to delete it
}

/*
Astar algorithm built from the Tutorial 2 on AI and from the
video at this link: https ://www.youtube.com/watch?v=KNXfSOx4eEE

*/
void Agent::executeSpatialAstar(Node start, Node finish){
	if (id == 4){
		int lakjsdg = 1;
	}
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
			if (!FindNodeAtList(adjacents[i], spatial_closedList)){
				if (!FindNodeAtList(adjacents[i], spatial_openList))
					spatial_openList.push_back(adjacents[i]);
			}
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

void Agent::move(){
	/*
	Step 3: Every t, advance one spot in the road.
	- If d has been reached or we reaced to the end of the road, stop;
	otherwise, continue advancing.
	*/
	
	if (stepsTaken < d){
		t++;
		if (actualNode == destination) {
			active = false; //we reached the end of the route 
		}
		else {
			actualNode = time_route[0];
			time_route.erase(time_route.begin());
		}
		stepsTaken++;
		map->setElement(actualNode.getX(), actualNode.getY(), id);

		cout << "Unit: " << id + 2 << " at location: " << actualNode.getX() << " , " << actualNode.getY();
		if (!active) cout << " This element is finished.";
		cout << endl;
	}
	else if(stepsTaken == d){
		if (id == 4){
			int something = 1;
		}
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
			if (actualNode == destination) active = false;
			t++;
		}
		/*
			This are cleared because the g, h, and f values of the nodes have changed, since now we
			are repathing from the perspective of the actualNode, not the beggining node.
		*/
		time_openList.clear(); 
		time_closedList.clear();
		time_route.clear(); //clear the route as a new route will start to be calculated
		actualNode.clearParent(); //Clear the parent, so this is a true starting point
		stepsTaken = 0;
		executeTimeSpaceAstar();
		move();
	}
	
}

void Agent::Reroute(int t){
	//Rerouting the agent
	//Step 1, delete everything from the current path starting on t
	//TODO: See if this element is going to be needed on the future
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
}

void Agent::TimeSpaceAstarHelper(Node start, Node finish){
	int time = t + 1;
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
	time_openList.push_back(A); //We put it on the open list
	Node P;
	
	
	/*
	Step 5: If you are in the end, stay there untl someone else needs your space.
	- In case that someone needs to pass, move n places away, then, star again calculating
	the spatial Astar.
	*/
	//Check if you already finished
	if (!active){
		/*
		TODO: Alert! Here we have a conflict to be solved by CBS. 
		what happens when an agent needs to use your space when you
		are finished moving
		*/
		
		//Set the cost of staying to 0
		A.setG(0);
		//If finished, stay the next seven steps where you are, unless someone needs to pass
		for (unsigned int i = 0; i < d; i++){
			//if someone needs to use your space
			if (map->isReserved(A, time, id)){
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
				tempD = d;
				d = time_route.size(); // so you can progress the next time
				replan = true;// so you can start pathfinding from that point
				break;// exit method;
			}

			//else
			else{
				//add to the route that you are staying in your place that period of time
				time_route.push_back(A); // push the elements, but dont reserve them so other entities can use them
				time++;
			}
		}
	}
	// If not finished, proceed to do the other normal stuff.
	else {

		int counter = d; //Thsi counter will help us find a partial path
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
			vector<Node> adjacents = getTimedAdjacents(P, time);

			for (unsigned int i = 0; i < adjacents.size(); i++){
				if (!FindNodeAtList(adjacents[i], time_closedList)){
					if (!FindNodeAtList(adjacents[i], time_openList))
						time_openList.push_back(adjacents[i]);
				}
			}
			std::sort(time_openList.begin(), time_openList.end());
			time++; //Increase time
			counter--; // Decrease the counter
			if (counter == 0){ //If the counter reached 0
				//Check for any partial path
				for (unsigned int i = 0; i < time_closedList.size(); i++){
					if (time_closedList[i].getDepth() >= d){
						// A partial path was found
						partial_path_nodes.push_back(time_closedList[i]);
					}
				}

				if (partial_path_nodes.size() > 0){ // If we have a partial path
					pathFound = true;
					break;
				}
				//Reset the counter in case no partial paths where found
				counter = d/2; //but make it smaller
			}
		}

		/*
		Step 2: Once the route has been found, reserve your path.
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

		
		// Now that we have a route, reserve it
		reserveRoute(t + 1);
	}

	
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

//Basically remove all the reserved adjacent nodes
std::vector<Node> Agent::getTimedAdjacents(Node element, int res_time){
	//First, get the adjacents of the node
	std::vector<Node> temp = map->adjacentHelper(element);

	//Then, check if they where reserved
	std::vector<Node> result;
	for (unsigned int i = 0; i < temp.size(); i++){
		if (!map->isReserved(temp[i], res_time, id)){
			result.push_back(temp[i]);
		}
	}

	//Now that we have all the available nodes, we calculate their real heuristic value
	for (unsigned int i = 0; i < result.size(); i++){
		
		calculateRealHeuristic(&result[i], destination);
		//calculate the rest
		result[i].setParent(element);
		result[i].calculateG();
		result[i].calculateF();
	}

	return result;
}






void Agent::setTime(int time_to_set){
	t = time_to_set;

	//Why dis??
	for (int i = 0; i < t; i++){
		time_route.push_back(actualNode);
	}
}



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
	executeSpatialAstar(actualNode, destination);
	time_route = spatial_route; // Because the route was saved on the spatial route
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
					for (unsigned int i = 0; i < c.t; i++){
						temp_path.push_back(time_route[i]);
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
	bool reserved = false;
	unsigned int reserved_index = 0;

	//If the route is smaller than d, just fill up the rest of the steps with your destination
	if (time_route.size() < d){
		Node n = time_route[time_route.size() - 1];
		while (time_route.size() < d){
			time_route.push_back(n);
		}
	}

	//Traverse the route untill d steps
	for (unsigned int i = 0; i < d; i++){

		//Check if the node is reserved for this given time
		if (map->isReserved(time_route[i], reservation_time, id)){
			//If it is reserved

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

		//Set the replan flag on
		replan = true;
		tempD = d;
		d = time_route.size();

	}
	else{
		/*
			Now that the route has been replaned, we can now remove all the elements after d
		*/
		while (time_route.size() > d){
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