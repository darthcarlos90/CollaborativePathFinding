#include "Agent.h"
/*
	Class: CBTNode
	Description: This class represents the nodes inside a Constraint Tree.
	Author: Carlos
*/

class CBTNode{
public:
	CBTNode(void); // When you are creating a root node
	CBTNode(vector<Constraint> parent_constraints, vector<Agent*> parents_agents); // When you are creating a child node, it inherits its dads nodes
	CBTNode(vector<Constraint> parent_constraints, vector<Conflict> conflicts, vector<Agent*> parents_agents); // When expanding a node with more than one conflict
	//Coment it for now, not sure if I will use it later
	//CBTNode(const CBTNode& n); //copy constructor
	~CBTNode(void);

	bool hasChildren() { return children.size() > 0; }
	int nChildren() { return children.size(); }
	CBTNode* childAt(int index) { return children[index]; }
	vector<CBTNode*> getChildren(){ return children; }
	
	CBTNode* getSmallestChild();

	void CalculatePaths(); // Calculates the paths of all the agents
	void ExpandNode(CBTNode n); // Expand a node when a conflict is found
	void validatePaths(); // Validates the paths of the elements
	

	bool isConsistent() { return consistent; }// all paths satisfy all constraints
	vector<Conflict> getConflicts() { return conflicts; }

	bool isValid() { return (conflicts.size() == 0); } // if it has conflicts
	

	int getCost() const { return cost; }
	void calculateCost();

	//CBTNode& operator = (const CBTNode& n); // Asigment operator
	bool operator < (const CBTNode& n);
	bool operator > (const CBTNode& n);

	void addAgent(Agent * a){ agents.push_back(a); }
	void addConstraint(Constraint c) { constraints.push_back(c); }




private:
	//Helper functions
	void findConstraint(int i);


	//Properties
	//Fix, this is not a binary tree
	vector<CBTNode*> children;
	int cost;
	
	vector<Agent*> agents; // The units
	vector<vector<Node>> paths; // The paths
	/*
		Note: I wonder if it is ok to save only the units, or to save the whole paths as well.
	*/

	vector<Constraint> constraints;
	bool consistent;
	vector<Conflict> conflicts;
	
};