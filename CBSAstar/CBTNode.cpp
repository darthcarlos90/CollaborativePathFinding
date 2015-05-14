#include "CBTNode.h"

CBTNode::CBTNode(void){
	cost = 0;
	consistent = true;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Agent*> agents){
	this->constraints = parent_constraints;
	this->agents = agents;
	cost = 0;
	consistent = true;
}

CBTNode::CBTNode(vector<Constraint> parent_constraints, vector<Conflict> conflicts, vector<Agent*> parents_agents){
	this->constraints = parent_constraints;
	this->conflicts = conflicts;
	this->agents = parents_agents;
	cost = 0;
	consistent = true;
}

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

void CBTNode::ExpandNode(CBTNode n){
	Conflict c = conflicts[0]; // Get the first conflict
	conflicts.erase(conflicts.begin()); // Delete the conflict being dealt with
	for (unsigned int i = 0; i < c.users.size(); i++){
		Constraint cnst(c.users[i], c.v, c.t);
		CBTNode* child = new CBTNode(constraints, conflicts, agents);
		child->addConstraint(cnst);
		children.push_back(child);
	}
}

void CBTNode::validatePaths(){
	unsigned int largest_size = 0;
	
	//Get the largest path size
	for (unsigned int i = 0; i < paths.size(); i++){
		if (paths[i].size() > largest_size) largest_size = paths[i].size();
	}

	for (unsigned int i = 0; i < largest_size; i++){
		//TODO: Finish this thing	
	}


}