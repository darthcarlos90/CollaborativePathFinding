#include "CBTNode.h"

CBTNode::CBTNode(void){
	cost = 0;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Agent*> parents_agents,vector<vector<Node>> parents_paths){
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
		agents[i]->calculateRoute();

		//Get the path, and push it into the paths vector
		paths.push_back(agents[i]->getPath());
	}
}

void CBTNode::ExpandNode(){
	//For catching erros
	if (conflict.empty){
		return;
	}

	for (unsigned int i = 0; i < conflict.users.size(); i++){
		Constraint cnst(conflict.users[i], conflict.v, conflict.t);
		CBTNode* child = new CBTNode(constraints, agents, paths);
		child->addConstraint(cnst);
		child->RecalculateRoutesOnConstraints();
		children.push_back(child);
	}
}

void CBTNode::validatePaths(){
	unsigned int largest_size = 0;
	
	//Get the largest path size
	for (unsigned int i = 0; i < paths.size(); i++){
		if (paths[i].size() > largest_size) largest_size = paths[i].size();
	}
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

bool CBTNode::findConstraintsConflicts(unsigned int t){
	bool foundConflict = false;
	vector<int> participateOnConflict;
	for (unsigned int toCompareId = 0; toCompareId < paths.size(); toCompareId++){
		// if the current path that we are analizing has an element on time t
		if (!foundConflict){
			if (paths[toCompareId].size() > t){
				Node toCompare = paths[toCompareId][t];
				Node toCompare1 = paths[toCompareId][t + 1];

				for (unsigned int i = toCompareId + 1; i < paths.size(); i++){
					//if the other element being analized, has an element on time t
					if (!foundConflict){
						if (paths[i].size() > t){
							if (toCompare == paths[i][t]){ // There is a conflict
								conflict.v = toCompare.getLocation();
								conflict.t = t;
								conflict.addUser(toCompareId);
								conflict.addUser(i);
								conflict.empty = false;
								foundConflict = true;
								
								if (!isAtList(toCompareId, participateOnConflict)) 
									participateOnConflict.push_back(toCompareId);
								if (!isAtList(i, participateOnConflict)) 
									participateOnConflict.push_back(i);
							} else if (toCompare == paths[i][t + 1] && toCompare1 == paths[i][t]){
								// We have a conflict
								//TODO: Solve this conflict
							}
						}
					}
					//Found a conflict, break
					else break;
				}
			}
		}
		//Found conflict, break
		else break;
	}

	//Now we need to create constraints with the elements that are not participating in conflicts
	std::sort(participateOnConflict.begin(), participateOnConflict.end());
	
	for (unsigned int i = 0; i < paths.size(); i++){
		if (participateOnConflict.size() > 0){
			if (i == participateOnConflict[0]){
				//This element is on conflict, no constraint needed
				//pop it out of the list
				participateOnConflict.erase(participateOnConflict.begin());

			}
			else {
				//Create a constraint and push it back into the list
				if (paths[i].size() > t){
					Constraint c(i, paths[i][t].getLocation(), t);
					Constraint c1(i, paths[i][t].getLocation(), t + 1); // Save that element for a time t + 1
					addConstraint(c);
					addConstraint(c1);
				}
			}
		}
		else {
			//I know its repeating, I just want to test it for now !!!!
			//Create a constraint and push it back into the list
			if (paths[i].size() > t){ // If there is actually a move at time t
				//save it
				Constraint c(i, paths[i][t].getLocation(), t);
				Constraint c1(i, paths[i][t].getLocation(), t + 1); // Save that element for a time t + 1
				addConstraint(c);
				addConstraint(c1);
			}
		}
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
		cost += agents[i]->getSic();
	}
}

void CBTNode::addAgent(Agent* a){
	//Set the id of the agent to its index on the list
	a->setId(agents.size());

	//Push the agent into the list
	agents.push_back(a);
}

void CBTNode::RecalculateRoutesOnConstraints(){
	for (unsigned int i = 0; i < agents.size(); i++){
		agents[i]->ModifyRouteOnConstraints(constraints);// Update the paths on the agent
		paths[i] = agents[i]->getPath(); // Let the node know the new paths
	}
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
		agents[i]->setPath(paths[i]);
	}
}