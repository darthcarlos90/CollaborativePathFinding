#include "CBTNode.h"

CBTNode::CBTNode(void){
	cost = 0;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Agent*> agents){
	this->constraints = parent_constraints;
	this->agents = agents;
	cost = 0;
	goal = false;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Conflict> conflicts, vector<Agent*> parents_agents){
	this->constraints = parent_constraints;
	this->conflicts = conflicts;
	this->agents = parents_agents;
	cost = 0;
	goal = false;
}

//Copy constructor
CBTNode::CBTNode(const CBTNode& n):
children(n.children), cost(n.cost), agents(n.agents),
paths(n.paths), constraints(n.constraints),
conflicts(n.conflicts)
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
	Conflict c = conflicts[0]; // Get the first conflict
	conflicts.erase(conflicts.begin()); // Delete the conflict being dealt with
	for (unsigned int i = 0; i < c.users.size(); i++){
		Constraint cnst(c.users[i], c.v, c.t);
		CBTNode* child = new CBTNode(constraints, conflicts, agents);
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
		findConstraintsConflicts(i);
	}

	if (conflicts.size() == 0){
		goal = true;
	}
}

void CBTNode::findConstraintsConflicts(int t){
	
	vector<int> participateOnConflict;
	for (unsigned int toCompareId = 0; toCompareId < paths.size(); toCompareId++){
		// if the current path that we are analizing has an element on time t
		if (paths[toCompareId].size() > t){
			Node toCompare = paths[toCompareId][t];

			for (unsigned int i = toCompareId + 1; i < paths.size(); i++){
				//if the other element being analized, has an element on time t
				if (paths[i].size() > t){
					if (toCompare == paths[i][t]){ // There is a conflict
						int conflictIndex = conflictAt(toCompare.getLocation());
						if (conflictIndex > -1){ //If the conflict index is bigger than -1, the conflict exists
							addAgentToConflict(conflictIndex, toCompareId);
							addAgentToConflict(conflictIndex, i);
						}
						else {
							Conflict c(toCompare.getLocation(), t);
							c.addUser(toCompareId);
							c.addUser(i);
							conflicts.push_back(c);
						}
						if (!isAtList(toCompareId, participateOnConflict)) participateOnConflict.push_back(toCompareId);
						if (!isAtList(i, participateOnConflict)) participateOnConflict.push_back(i);
					}
				}
			}
		}
	}

	//Now we need to create constraints with the elements that are not participating in conflicts
	//TODO: Beware on errors at debug on here :D
	std::sort(participateOnConflict.begin(), participateOnConflict.end());
	for (unsigned int i = 0; i < paths.size(); i++){
		if (i == participateOnConflict[0]){
			//This element is on conflict, no constraint needed
			//pop it out of the list
			participateOnConflict.erase(participateOnConflict.begin());
		}
		else {
			//Create a constraint and push it back into the list
			Constraint c(i, paths[i][t].getLocation(), t);
			constraints.push_back(c);
		}
	}

}

void CBTNode::addAgentToConflict(int index, int id){
	bool found = false;
	for (unsigned int i = 0; i < conflicts[index].users.size(); i++){
		if (conflicts[index].users[i] == id){
			found = true;
			break;
		}
	}

	if (!found) conflicts[index].users.push_back(id);
}

int CBTNode::conflictAt(Location location){
	bool result = -1;
	if (conflicts.size() == 0) return result;
	else {
		for (unsigned int i = 0; i < conflicts.size(); i++){
			if (conflicts[i].v == location){
				result = i;
				break;
			}
		}
	}

	return result;
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
	for (int i = 0; i < agents.size(); i++){
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
		agents[i]->ModifyRouteOnConstraints(constraints);
	}
}