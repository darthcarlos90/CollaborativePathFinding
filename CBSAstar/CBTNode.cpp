#include "CBTNode.h"

CBTNode::CBTNode(void){
	cost = 0;
	goal = false;
	swapcounter = 0;
	main_actor_id = 0;
	validNode = true;
	CATCost = 0;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Agent> parents_agents,vector<vector<Node>> parents_paths){
	this->constraints = parent_constraints;
	this->agents = parents_agents;
	this->paths = parents_paths;
	cost = 0;
	goal = false;
	SanitizePaths();
	UpdateCAT();
	BalancePaths();
	swapcounter = 0;
	main_actor_id = 0;
	validNode = true;
	CATCost = 0;
}

//Copy constructor
CBTNode::CBTNode(const CBTNode& n):
children(n.children), cost(n.cost), agents(n.agents),
paths(n.paths), constraints(n.constraints),
conflict(n.conflict), swapcounter(n.swapcounter),
main_actor_id(n.main_actor_id)
{}

CBTNode::~CBTNode(void){
	//No need for a destructor for now
	//TODO: Check if this is needed
}

bool CBTNode::operator < (const CBTNode& n){
	/*
		TODO: Delete the next code an paste whats next if something stops working.
		return (this->cost < n.getCost());
		Explanation:
		Because there will be times when two nodes will be the same, the tie break will be decided by how 
		many possible conflicts does a node has. This will be created with the help of the CAT table.
	*/
	if (this->cost == n.getCost()){
		return (this->getCATCost() < n.getCATCost());
	
	}
	else return (this->cost < n.getCost());
	
}

bool CBTNode::operator >(const CBTNode& n){
	return (this->cost > n.getCost());
}

CBTNode* CBTNode::getSmallestChild(){
	bool priorityAgentExist = false;
	int index_smaller = 0;
	int priorityId = -1;

	// If there is one agent with priority, priorityAgentExists will be true
	for (unsigned int i = 0; i < agents.size(); i++){
		if (agents[i].getPriority()) priorityId = i;
		priorityAgentExist = priorityAgentExist || agents[i].getPriority();
	}

	for (unsigned int i = 0; i < children.size(); i++){
		// If there are any invalid nodes, ignore priority
		if (!children[i]->isValidNode()){
			for (unsigned int i = 0; i < agents.size(); i++){
				agents[i].setPriority(false); // deactivate priorities
			}
			priorityAgentExist = false;
			break;
		}
	}

	if (priorityAgentExist && swapcounter > 2 && children.size() == 4){
		// First, find the first child node that has that actor as its main actor
		for (unsigned int i = 0; i < children.size(); i++){
			if (children[i]->getMainActor() == priorityId){
				index_smaller = i;
				break;
			}
		}

		// Now, find the smaller amon the ones with the priority
		for (unsigned int i = index_smaller + 1; i < children.size(); i++){
			if (*children[i] < *children[index_smaller]){
				index_smaller = i;
			}
		}
	}
	else {
		// First, look for the first element that has a valid solution
		for (unsigned int i = 0; i < children.size(); i++){
			if (children[i]->isValidNode()){
				index_smaller = i;
				break;
			}
		}

		// Now compare it against valid solutions with cost
		for (unsigned int i = 0; i < children.size(); i++){
			if (i != index_smaller){
				if ((*children[i] < *children[index_smaller]) && children[i]->isValidNode()){
					index_smaller = i;
				}
			}
		}
	}
	
	
	

	return children[index_smaller];
}

void CBTNode::CalculatePaths(){
	for (unsigned int i = 0; i < agents.size(); i++){
		//Calculate the paths of each of the agents, as if they where the only element on the grid
		agents[i].calculateRoute();
		validNode = validNode && agents[i].hasValidSolution();
		//Get the path, and push it into the paths vector
		paths.push_back(agents[i].getPath());
	}

	
	UpdateCAT();
}

void CBTNode::ExpandNode(){
	/*
		If code enters this part, either an error, but most likely, the node has no
		solution.
	*/
	if (conflict.empty){
		return;
	}

	int childCost = 0;

	for (unsigned int i = 0; i < conflict.users.size(); i++){
		if (conflict.times[i] != 0){
			Constraint cnst(conflict.users[i], conflict.locations[i], conflict.times[i]);
			CBTNode* child = new CBTNode(constraints, agents, paths);
			child->addConstraint(cnst);
			child->RecalculateRoutesOnConstraints(conflict.users[i], conflict.replan_flag);
			child->calculateCost();
			child->setSwapCounter(swapcounter);
			childCost += child->getCost();
			children.push_back(child);
		}
		else {
			//cout << "We have a constraint at a time 0." << endl;
		}
	}

	// If the children couldnt find a route, then discard this node
	if (childCost == 0){
		validNode = false;
		this->cost = 0;
	}

	DestroyCycle();
}

void CBTNode::DestroyCycle(){
	int size = BalancePaths();
	int mapsize = agents[0].getMapNodes();
	if (size > (mapsize)){
		validNode = false;
		this->cost = 0;
	}
}

void CBTNode::validatePaths(){
	if (validNode){
		//Get the largest path size
		unsigned int largest_size = BalancePaths();
		/*
			We use the index i as the time, the first element, the one located at
			t = 0, is the starting point, so that element cant be checked for
			obvious reasons, that is why the checking starts at 1. NOT
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
}

/*
	Checks how many possible conflicts there are in the designated child, and balances it through the cost
*/
void CBTNode::countPossibleConflicts(){
	unsigned int largest_size = BalancePaths();
	int count = 0;
	for (unsigned int i = 0; i < largest_size; i++){
		if (findConstraintsConflicts(i)){
			count++;
		}
	}

	conflict.clear();
	//Clear the conflict!!!!

	cost += count * 10;
	//SanitizePaths();
}

void CBTNode::CreateConflict(unsigned int time_ocurrence, Location location, vector<int> users){
	conflict.empty = false;
	conflict.replan_flag = false;

	bool blockingConflict = false;
	int blockingUser = -1;
	
	for (unsigned int i = 0; i < users.size(); i++){
		conflict.times.push_back(time_ocurrence);
		conflict.locations.push_back(location);
		if (location == agents[users[i]].getDestinationLocation()){
			if (agents[users[i]].debugNumberOfAdjacents()){
				/*
					If we have the case where the destination is blocking other elements, and this destination is on a 
					tight place (that is why the adjacents is 2 or less), then this element is blocking other elements,
					thus this element MUST MOVE.
				*/
				blockingConflict = true;
				blockingUser = i;
				break;
			}
		}
	}

	if (blockingConflict){
		conflict.times.clear();
		conflict.locations.clear();
		conflict.times.push_back(time_ocurrence);
		conflict.locations.push_back(location);
		conflict.users.push_back(users[blockingUser]);
		//conflict.replan_flag = true;
	}
	else {
		conflict.users = users;
	}

	
	swapcounter = 0; // reset the swap counter
}

void CBTNode::CreateSpecialConflict(unsigned int time, vector<Location> locations, vector<int> users){
	conflict.replan_flag = true;
	conflict.empty = false;
	

	bool blockingConflict = false;
	int blockingEntity = -1;

	Location destination1 = agents[users[0]].getDestinationLocation();
	Location destination2 = agents[users[1]].getDestinationLocation();

	bool comparison1 = (locations[1] == destination1) || (locations[0] == destination1);
	bool comparison2 = (locations[1] == destination2) || (locations[0] == destination2);

	// This is the cause of all evil
	if (comparison1 && comparison2){
		// If both are trying to get to their destination, stop, should let the other one pass first
		for (unsigned int i = 0; i < users.size(); i++){
			Location destination_location = agents[users[i]].getDestinationLocation();
			Location otherLocation;
			conflict.users.push_back(users[i]);
			// Check which location is not destination and push that
			for (unsigned int j = 0; j < locations.size(); j++){
				if (!(destination_location == locations[j])){
					otherLocation = locations[j];
					conflict.locations.push_back(otherLocation);
				}
					
			}
			
			// Now that we know what location must not be occupied, push the time at where they must not occupy it
			if (agents[users[i]].getLocationAtTime(time) == otherLocation){
				conflict.times.push_back(time);
			}
			else conflict.times.push_back(time + 1);
		}
		

		swapcounter++;

	}
	else{

		for (unsigned int i = 0; i < users.size(); i++){
			//Add the corresponding times
			if (time != 0){
				conflict.times.push_back(time);
				conflict.users.push_back(users[i]);
			}

			conflict.times.push_back(time + 1);
			// Add twice every user
			conflict.users.push_back(users[i]);


			// TODO: uncomment or comment if necesary, lets just assume you cant occupy before the destination
			if ((locations[1] == agents[users[i]].getDestinationLocation()) || (locations[0] == agents[users[i]].getDestinationLocation())){
				if (agents[users[i]].debugNumberOfAdjacents()){
					blockingConflict = true;
					blockingEntity = i;
					break;
				}

			}

		}

		if (blockingConflict){
			conflict.times.clear();
			conflict.locations.clear();
			conflict.users.clear();

			Location destination = agents[users[blockingEntity]].getDestinationLocation();
			
			if ((agents[users[blockingEntity]].getLocationAtTime(time) == destination) && time != 0) {
				conflict.times.push_back(time);
			} else conflict.times.push_back(time + 1);
			

			conflict.users.push_back(users[blockingEntity]);

			/*if (blockingEntity == 0){
				conflict.locations.push_back(locations[0]);
				conflict.locations.push_back(locations[1]);
			}
			else {
				conflict.locations.push_back(locations[1]);
				conflict.locations.push_back(locations[0]);
			}*/
			if (time != 0){
				if (locations[0] == destination){
					conflict.locations.push_back(locations[0]);
				}
				else conflict.locations.push_back(locations[1]);
			}
			else {
				if (locations[0] == destination){
					conflict.locations.push_back(locations[1]);
				}
				else conflict.locations.push_back(locations[0]);
			}
			
			swapcounter = 0;
			
			
		}
		else {
			// Add the locations in the correct order
			if (time != 0) conflict.locations.push_back(locations[0]);
			conflict.locations.push_back(locations[1]);
			if (time != 0) conflict.locations.push_back(locations[1]);
			conflict.locations.push_back(locations[0]);
			swapcounter++; // Increase the swap counter, since this is a swap


			
		}
		
	}

	
	
}

void CBTNode::CreateDestinationConflict(unsigned int time, Location location, int user){
	conflict.users.push_back(user);
	conflict.times.push_back(time);
	conflict.locations.push_back(location);
	conflict.replan_flag = false;
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

						// create the conflict with the information
						CreateConflict(t, toCompare, users);
						// flag to break the search
						foundConflict = true;
						break;

					}
					
					else if (t + 1 < paths[i].size()){
						/*
							Fix: Can't explain this in english. Antes, detectabamos si un agente
							queria ocupar un nodo n en el tiempo t + 1, cuando n estaba siendo ocupado
							en el tiempo t. Este movimiento es valido, sin embargo aqui lo marcamos como
							invalido, lo cual lleva a que ciertos elementos fallen. Por esa razon, se 
							cambiara a la deteccion total de dos elementos colisionando frente a frente.
							Date: 22/07/2015
							Why? Google translate the above.
						*/
						if (toCompare == paths[i][t + 1].getLocation()){
							// We probably have a new conflict, lets test the other elments
							if (paths[toCompareId][t + 1] == paths[i][t]){
								// We have another type of conflict, an invalid movement!
								/*
									On this kind of conflict there will be 4 nodes created:
									- One where agent 1 can't occupy the node at t
									- One for t + 1
									- And the same for the other agent
								*/
								vector<int> users;
								users.push_back(toCompareId);
								users.push_back(i);
								vector<Location> locations;
								locations.push_back(toCompare);
								locations.push_back(paths[i][t].getLocation());
								CreateSpecialConflict(t, locations, users);
								foundConflict = true;
								break;
							}
							
							
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
		validNode = validNode && agents[i].hasValidSolution();
	}


	if (validNode){
		BalancePaths();
		//SanitizePaths();
		for (unsigned int i = 0; i < agents.size(); i++){
			cost += agents[i].getSic();
			validNode = validNode && agents[i].hasValidSolution();
		}
	}

	CalculateCATCost();

	//countPossibleConflicts();
}

// Adds and returns the id of the element that was recently added
int CBTNode::addAgent(Agent a){
	//Set the id of the agent to its index on the list
	a.setId(agents.size());

	//Push the agent into the list
	agents.push_back(a);

	return a.getId();
}

void CBTNode::RecalculateRoutesOnConstraints(int agent_id, bool replan_flag){
	agents[agent_id].ModifyRouteOnConstraints(constraints, replan_flag, CAT);// Update the paths on the agent
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
	
	// The main actor id gotten from the new constraint
	main_actor_id = c.id;
}

void CBTNode::UpdateAgentsPaths(){
	if (validNode){
		for (unsigned int i = 0; i < agents.size(); i++){
			agents[i].setPath(paths[i]);
			agents[i].setValidPath(true);
		}
		SanitizePaths();
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
	SanitizePaths();
	UpdateCAT();

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
				toAdd.calculateManhattanHeuristic(agents[i].getDestinationLocation());
				toAdd.setG(agents[i].getPath()[agents[i].pathSize() - 1].getG() + 1);
				toAdd.calculateF();
				agents[i].PushElementAtTheBackOfRoute(toAdd);
			}
			// Update the paths vector
			paths[i] = agents[i].getPath();
		}
	}

	return largestSize;
}

void CBTNode::SanitizePaths(){
	for (unsigned int i = 0; i < agents.size(); i++){
		// Sanitize the path at agent level
		agents[i].SanitizePath();
		// calculate the cost of this agent
		agents[i].calculateSIC();
		// save the path
		paths[i] = agents[i].getPath();
	}
}

void CBTNode::UpdateCAT(){
	CAT.clear();
	for (unsigned int i = 0; i < paths.size(); i++){
		for (unsigned int j = 1; j < paths[i].size(); j++){
			CAT.push_back(Constraint(agents[i].getId(), paths[i][j].getLocation(), j));
		}
	}
}

void CBTNode::setSwapCounter(int val){
	swapcounter = val;
	/*
		If there are more than 2 consecutive swaps in the problem, it means
		that we are trying to solve a corridor. So, when we are trying to solve a corridor,
		the most optimal way of solving it is: one agent moves out of the way, the other
		gets to its position, and then the other one goes back to solving its problem.
		For this, we are going to set an agent with a priority. This priority will indicate
		who gets to move out of the way, and who doesnt. (If priority = true, you move out of the way).
		So when taking on decisions, you will always choose when one of the agents gets to move.
	*/
	if (swapcounter > 2){
		// create an array to tll the agents appearances in constraint table
		int *appearancesConstraintTable = new int[agents.size()];

		// Set the appearances to 0
		for (unsigned int i = 0; i < agents.size(); i++){
			appearancesConstraintTable[i] = 0;
		}

		// Count the appareances
		for (unsigned int i = 0; i < constraints.size(); i++){
			appearancesConstraintTable[constraints[i].id]++;
		}

		int indexBigger = 0;
		for (unsigned int i = 1; i < agents.size(); i++){
			if (appearancesConstraintTable[indexBigger] < appearancesConstraintTable[i]){
				indexBigger = i;
			}
		}

		// Once finished, we set the priority to that element
		if (validNode) agents[indexBigger].setPriority(true);

		delete appearancesConstraintTable;
		appearancesConstraintTable = NULL;

	}
}


void CBTNode::CalculateCATCost() {
	/*
		Lambda function because YOLO.
	*/
	std::function<int(Location, vector<Constraint>)> FindNumberOcurrancesCAT = [](Location location, vector<Constraint> CAT) {
		int result = 0;
		for (unsigned int i = 0; i < CAT.size(); i++){
			if (CAT[i].location == location) result++;
		}

		return result;
	};

	
	for (unsigned int i = 0; i < paths.size(); i++){
		for (unsigned int index = 0; index < paths[i].size(); index++){
			CATCost += FindNumberOcurrancesCAT(paths[i][index].getLocation(), CAT);
		}
	}

	
}