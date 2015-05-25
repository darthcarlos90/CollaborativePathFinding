#include "Agent.h"
/*
	Class: CBTNode
	Description: This class represents the nodes inside a Constraint Tree.
	Author: Carlos
*/

class CBTNode{
public:
	CBTNode(void); // When you are creating a root node
	CBTNode(vector<Constraint> parent_constraints, vector<Agent*> parents_agents, vector<vector<Node>> parents_paths); // When you are creating a child node, it inherits its dads nodes
	CBTNode(const CBTNode& n); //copy constructor
	~CBTNode(void);

	bool hasChildren() { return children.size() > 0; }
	int nChildren() { return children.size(); }
	CBTNode* childAt(int index) { return children[index]; }
	vector<CBTNode*> getChildren(){ return children; }
	
	CBTNode* getSmallestChild();

	void CalculatePaths(); // Calculates the individual paths of all the agents
	void ExpandNode(); // Expand a node when a conflict is found
	void validatePaths(); // Validates the paths of the elements
	

	Conflict getConflict() const { return conflict; }

	bool isValid() { return conflict.empty; } // if it has conflicts
	bool isGoal() { return goal; } // Check if the node is a goal node
	

	int getCost() const { return cost; }
	void calculateCost(); // calculates the cost of the node

	//CBTNode& operator = (const CBTNode& n); // Asigment operator
	bool operator < (const CBTNode& n);
	bool operator > (const CBTNode& n);

	void addAgent(Agent * a);
	Agent* getAgent(int index){ return agents[index]; }
	void AddPath(vector<Node> value) { paths.push_back(value); } // For precalculated paths
	
	void addConstraint(Constraint c);

	void RecalculateRoutesOnConstraints();

	void UpdateAgentsPaths(); // This method updates the path of their agents



private:
	//Helper functions
	//void findConstraint(int i); // Forgot what is this for, Ill coment it until I remember why this is here
	bool findConstraintsConflicts(unsigned int t); //Finds the constraints and conflictsd at time t
	bool isAtList(int element, vector<int> list); //Looks for a int on a list of ints, if it is there, returns true


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
	Conflict conflict;

	bool goal;
	
};