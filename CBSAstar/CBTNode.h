#include "Agent.h"
/*
	Class: CBTNode
	Description: This class represents the nodes inside a Constraint Tree.
	Author: Carlos
*/

class CBTNode{
public:
	CBTNode(void); // When you are creating a root node
	CBTNode(vector<Constraint> parent_constraints, vector<Agent> parents_agents, vector<vector<Node>> parents_paths); // When you are creating a child node, it inherits its dads nodes
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

	int addAgent(Agent a);
	Agent getAgent(int index){ return agents[index]; }
	void AddPath(vector<Node> value) { paths.push_back(value); } // For precalculated paths
	
	void addConstraint(Constraint c);

	void RecalculateRoutesOnConstraints(bool dest_conf, int agent_id);

	void UpdateAgentsPaths(); // This method updates the path of their agents

	//TODO: Create necesary methods for the reroute of the elements
	//Necesary methods for replanification of routes
	/*
		This method replans from the last element in the agents path, till
		its destination, taking into account the other paths from the other
		agents in the node.
		Return an int, that int is the index of the path where it will start rerouting.
	*/
	int ReplanAgentFromLastIndex(int agentId);

	/*
		This method will repeat the element at the index right after the one
		stated on the parameters.
	*/
	void WaitAtIndex(int id, int index, int times);

	/*
		There are times, when the agent is created null. Even though the path
		is calculated correctly, the agets yet remain null, thus acces to the agents
		produces errors.
		TODO: Fix that shit
		This method grants acces to the paths, BUT, it must be fixed
	*/
	vector<Node> getPathAt(unsigned int index);

	// Copy paste of a method used in another class, since I could find a more efficient way to 
	// reuse that code
	void FindSpecialCases();

	

	

private:
	//Helper functions
	//void findConstraint(int i); // Forgot what is this for, Ill coment it until I remember why this is here
	bool findConstraintsConflicts(unsigned int t); //Finds the constraints and conflictsd at time t
	bool isAtList(int element, vector<int> list); //Looks for a int on a list of ints, if it is there, returns true
	bool LocationAtNodeList(Location location, vector<Node> list, int* index = NULL);
	void CreateConflict(unsigned int time_ocurrence, Location location, vector<int> users, bool dest_con);
	bool FindDeadLock();
	void SolveDeadLock();

	
	// This method balances the paths of the agents so a proper check coudld be done
	int BalancePaths();


	//Properties
	//Fix, this is not a binary tree
	vector<CBTNode*> children;
	int cost;
	
	vector<Agent> agents; // The units
	vector<vector<Node>> paths; // The paths
	/*
		Note: I wonder if it is ok to save only the units, or to save the whole paths as well.
	*/

	vector<Constraint> constraints;
	Conflict conflict;

	Conflicted deadlock; // Any deadlock detected will be stored here

	bool goal;
	
};