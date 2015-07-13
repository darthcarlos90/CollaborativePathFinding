#include "CBTNode.h"

CBTNode::CBTNode(void){
	cost = 0;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Agent> parents_agents,vector<vector<Node>> parents_paths){
	this->constraints = parent_constraints;
	this->agents = parents_agents;
	this->paths = parents_paths;
	cost = 0;
	goal = false;
}

//Copy constructor
CBTNode::CBTNode(const CBTNode& n):
children(n.children), cost(n.cost), agents(n.agents),
paths(n.paths), constraints(n.constraints),
conflict(n.conflict)
{}

CBTNode::~CBTNode(void){
	//No need for a destructor for now
}

bool CBTNode::operator < (const CBTNode& n){
	return (this->cost < n.getCost());
}

bool CBTNode::operator >(const CBTNode& n){
	return (this->cost > n.getCost());
}

CBTNode* CBTNode::getSmallestChild(){
	int index_smaller = 0;
	for (unsigned int i = 1; i < children.size(); i++){
		if (*children[i] < *children[index_smaller]){
			index_smaller = i;
		}
	}

	return children[index_smaller];
}

void CBTNode::CalculatePaths(){
	for (unsigned int i = 0; i < agents.size(); i++){
		//Calculate the paths of each of the agents, as if they where the only element on the grid
		agents[i].calculateRoute();

		//Get the path, and push it into the paths vector
		paths.push_back(agents[i].getPath());
	}
}

void CBTNode::ExpandNode(){
	//For catching erros
	if (conflict.empty){
		return;
	}

	for (unsigned int i = 0; i < conflict.users.size(); i++){
		Constraint cnst(conflict.users[i], conflict.v, conflict.times[i]);
		CBTNode* child = new CBTNode(constraints, agents, paths);
		bool dest_conf = conflict.destination_conflict;
		child->addConstraint(cnst);
		child->RecalculateRoutesOnConstraints(dest_conf, conflict.users[i]);
		child->calculateCost();
		children.push_back(child);
	}
}

void CBTNode::validatePaths(){
	unsigned int largest_size = BalancePaths();
	//Get the largest path size
	// Uneeded but still 
	/*for (unsigned int i = 0; i < paths.size(); i++){
		if (paths[i].size() > largest_size) largest_size = paths[i].size();
	}*/
	/*
		We use the index i as the time, since the first elements of the path will be executed on 
		t = 0, the second element on t = 1 and so on
	*/
	for (unsigned int i = 0; i < largest_size; i++){
		if (findConstraintsConflicts(i)){
			break;
		}
	}

	if (conflict.empty){
		goal = true;
	}
}

void CBTNode::CreateConflict(unsigned int time_ocurrence, Location location, vector<int> users, bool dest_con){
	conflict.v = location;
	conflict.times.push_back(time_ocurrence);
	conflict.times.push_back(time_ocurrence);
	conflict.users = users;
	conflict.destination_conflict = dest_con;
	conflict.empty = false;
}

void CBTNode::CreateSpecialConflict(vector<unsigned int> times, Location location, vector<int> users){
	conflict.v = location;
	conflict.times = times;
	conflict.users = users;
	conflict.empty = false;
}

bool CBTNode::findConstraintsConflicts(unsigned int t){
	bool foundConflict = false;

	for (unsigned int toCompareId = 0; toCompareId < paths.size(); toCompareId++){
		if (!foundConflict){
			Location toCompare = paths[toCompareId][t].getLocation();
			// To use the other player
			for (unsigned int i = 0; i < paths.size(); i++){
				if (toCompareId != i){ // If different players
					if (toCompare == paths[i][t].getLocation()){ // There is a conflict
						vector<int> users;
						users.push_back(toCompareId);
						users.push_back(i);
						//Se how much more agents are invovled in this conflict
						for (unsigned int j = i + 1; j < paths.size(); j++){
							if (toCompare == paths[j][t].getLocation())
								users.push_back(j);
						}

						// create the conflict with the information
						CreateConflict(t, toCompare, users, false);
						// flag to break the search
						foundConflict = true;
						break;

					}
					else if (t + 1 < paths[i].size()){
						if (toCompare == paths[i][t + 1].getLocation()){
							// We have another type of conflict, an invalid movement!
							vector<int> users;
							users.push_back(toCompareId);
							users.push_back(i);
							vector<unsigned int> times;
							times.push_back(t);
							times.push_back(t + 1);
							CreateSpecialConflict(times, toCompare, users);
							foundConflict = true;
							break;
						}
					}
				}
			}
		}
		//Found conflict, break
		else break;
	}

	return foundConflict;

}

bool CBTNode::isAtList(int element, vector<int> list){
	if (!list.empty()){
		if (std::find(list.begin(), list.end(), element) != list.end()){
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

void CBTNode::calculateCost(){
	for (unsigned int i = 0; i < agents.size(); i++){
		cost += agents[i].getSic();
	}
}

// Adds and returns the id of the element that was recently added
int CBTNode::addAgent(Agent a){
	//Set the id of the agent to its index on the list
	a.setId(agents.size());

	//Push the agent into the list
	agents.push_back(a);

	return a.getId();
}

void CBTNode::RecalculateRoutesOnConstraints(bool dest_conf, int agent_id){
	agents[agent_id].ModifyRouteOnConstraints(constraints, dest_conf);// Update the paths on the agent
	paths[agent_id] = agents[agent_id].getPath(); // Let the node know the new paths
	agents[agent_id].calculateSIC();
}

void CBTNode::addConstraint(Constraint c){
	
	if (!constraints.empty()){
		if (std::find(constraints.begin(), constraints.end(), c) != constraints.end()){
			//The element is found, do nothing
		}
		else{
			//The constraint is not found, push it to the vector
			constraints.push_back(c);
		}
	}
	else{
		//The list is empty, add the element
		constraints.push_back(c);
	}
}

void CBTNode::UpdateAgentsPaths(){
	for (unsigned int i = 0; i < agents.size(); i++){
		agents[i].setPath(paths[i]);
	}
}

int CBTNode::ReplanAgentFromLastIndex(int agentId){
	int result = agents[agentId].getPath().size() - 1;
	agents[agentId].ReroutePathUsingCBS();
	paths[agentId] = agents[agentId].getPath(); // Update the path in the tree
	return result;

}

void CBTNode::WaitAtIndex(int id, int index, int times){
	agents[id].RepeatStepAtIndex(index, times);
	paths[id] = agents[id].getPath(); // Update the paths otherwise they will get overriden
}

vector <Node> CBTNode::getPathAt(unsigned int index){
	return paths[index]; 
}


void CBTNode::FindSpecialCases(){
	// Just for Now
	if (FindDeadLock()) SolveDeadLock();
}

bool CBTNode::FindDeadLock(){
	bool result = false;
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
									result = true;
									deadlock.agents.push_back(toCompare);
									deadlock.agents.push_back(index);
									break;
								}
							}
						}
					}
				}
				if (result) break;
			}
			if (result) break;
		}
		if (result) break;
	}
	
	return result;
}


void CBTNode::SolveDeadLock(){
	int agent1 = deadlock.agents[0];
	int agent2 = deadlock.agents[1];
	int hightPriorityIndex;
	int lowPriorityIndex;

	bool partialDestinationElement = false;

	// For easy coding, pointers
	Agent* highPriority = NULL;
	Agent* lowPriority = NULL;

	// If both have partial destination or if none has partial destination
	if ((agents[agent1].hasPartialDestination() && agents[agent2].hasPartialDestination()) || 
		(!agents[agent1].hasPartialDestination() && !agents[agent2].hasPartialDestination())){

		int difference = abs(static_cast<int>(paths[agent1].size()) - static_cast<int>(paths[agent2].size()));
		// Making waiting priority by size, the agent with smallest path waits ...
		if (difference > 5){
			if (paths[agent1] < paths[agent2]){
				highPriority = &agents[agent2];
				lowPriority = &agents[agent1];
				hightPriorityIndex = agent2;
				lowPriorityIndex = agent1;
			}
			else {
				highPriority = &agents[agent1];
				hightPriorityIndex = agent1;
				lowPriority = &agents[agent2];
				lowPriorityIndex = agent2;
			}

		}
		else {
			// Else priority is dictated by the position of their starting position
			if (LocationAtNodeList(agents[agent1].getLocation(), paths[agent2])){
				highPriority = &agents[agent1];
				hightPriorityIndex = agent1;
				lowPriority = &agents[agent2];
				lowPriorityIndex = agent2;
			}
			else{
				highPriority = &agents[agent2];
				lowPriority = &agents[agent1];
				hightPriorityIndex = agent2;
				lowPriorityIndex = agent1;
			}
		}
	}
	// If only one has a partial destination
	else{
		partialDestinationElement = true; // A partial destination was found, calculate stuff here
		// I know , repetition, but I just want to see if it works and then Ill fix this
		if (agents[agent1].hasPartialDestination()){
			highPriority = &agents[agent1];
			hightPriorityIndex = agent1;
			lowPriority = &agents[agent2];
			lowPriorityIndex = agent2;
		}
		else{
			highPriority = &agents[agent2];
			lowPriority = &agents[agent1];
			hightPriorityIndex = agent2;
			lowPriorityIndex = agent1;
		}

		int partialDestinationindex = 0;
		Location partialDestinationLocation = highPriority->getPartialDestination().getLocation();
		// Get at what index is the partial destination of the priority agent
		for (unsigned int i = 0; i < paths[hightPriorityIndex].size(); i++){
			if (paths[hightPriorityIndex][i].getLocation() == partialDestinationLocation){
				partialDestinationindex = i;
				break;
			}
		}

		// Save the size of the low priority element.
		int lowPriorityPathSize = lowPriority->pathSize();

		/*
			Now, make the low priority element wait until the high priority element reaches
			their partial destination.
		*/
		for (int i = 0; i < partialDestinationindex; i++){
			lowPriority->AddNodeToPathAtTimeT(lowPriority->getActualLocation(), 0);
		}

		// Now, make the high priority element wait on the escape node while low priority finishes
		for (int i = 0; i < lowPriorityPathSize - 1; i++){
			highPriority->AddNodeToPathAtTimeT(highPriority->getPartialDestination(), partialDestinationindex);
		}

		//Update paths
		paths[lowPriorityIndex] = lowPriority->getPath();
		paths[hightPriorityIndex] = highPriority->getPath();

	}
	
	if (!partialDestinationElement){ // If no partial destination was found ...
		// The priority element must calculate its route to it's destination normally
		highPriority->calculateRoute();
		// Update the route
		paths[hightPriorityIndex] = highPriority->getPath();

		// We need to know the amount of steps it took to the other element to get to their destination
		int highPriorityPathSize = highPriority->pathSize();

		// First, calculate the route as if it was the only element
		lowPriority->calculateRoute();

		// If the actual position of the low priority is part of the high priority
		int index = 0; // save the index
		if (LocationAtNodeList(lowPriority->getLocation(), paths[hightPriorityIndex], &index)){
			for (int i = 0; i < index; i++){
				lowPriority->AddNodeToPathAtTimeT(lowPriority->getActualLocation(), 0);
			}
		}
		else {
			// Make the low priority element wait in its starting position until high priority element is finished
			for (int i = 0; i < highPriorityPathSize; i++){
				lowPriority->AddNodeToPathAtTimeT(lowPriority->getActualLocation(), 0);
			}
		}

		// Update the route
		paths[lowPriorityIndex] = lowPriority->getPath();

	}

}

bool CBTNode::LocationAtNodeList(Location location, vector<Node> list, int * index){
	bool result = false;
	for (unsigned int i = 0; i < list.size(); i++){
		if (location == list[i].getLocation()){
			if (index) *index = i;
			result = true;
			break;
		}
	}

	return result;
}

int CBTNode::BalancePaths(){
	unsigned int largestSize = 0;
	// Get the largest pathsize
	for (unsigned int i = 0; i < agents.size(); i++){
		if (agents[i].pathSize() > largestSize) 
			largestSize = agents[i].pathSize();
	}

	// Update all the paths so they are all the same size
	for (unsigned int i = 0; i < agents.size(); i++){
		if (agents[i].pathSize() != largestSize){
			int difference = largestSize - agents[i].pathSize();
			for (int j = 0; j < difference; j++){
				// Push to the back of the route the element of the last index
				// First calculate the correct values in case of need
				Node toAdd = agents[i].getPath()[agents[i].pathSize() - 1];
				toAdd.setG(agents[i].getPath()[agents[i].pathSize() - 1].getG() + 10);
				toAdd.calculateF();
				agents[i].PushElementAtTheBackOfRoute(toAdd);
			}
			// Update the paths vector
			paths[i] = agents[i].getPath();
		}
	}

	return largestSize;
}