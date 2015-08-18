#include "Agent.h"
#include <functional>
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

	void RecalculateRoutesOnConstraints(int agent_id, bool replan_flag);

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


	void setSwapCounter(int val);
	int getSwapCounter() { return swapcounter; }

	int getMainActor() { return main_actor_id; }

	// If this node could find a solution
	bool isValidNode() { return validNode; }
	
	int getCATCost() const { return CATCost; }

	// This method balances the paths of the agents so a proper check coudld be done
	int BalancePaths();

	// Get the number of agents in this node
	int NumberAgents() { return agents.size(); }
	

private:
	//Helper functions
	//void findConstraint(int i); // Forgot what is this for, Ill coment it until I remember why this is here
	bool findConstraintsConflicts(unsigned int t); //Finds the constraints and conflictsd at time t
	bool isAtList(int element, vector<int> list); //Looks for a int on a list of ints, if it is there, returns true
	bool LocationAtNodeList(Location location, vector<Node> list, int* index = NULL);
	void CreateConflict(unsigned int time_ocurrence, Location location, vector<int> users);
	void CreateSpecialConflict(unsigned int time, vector<Location> locations, vector<int> users);
	void CreateDestinationConflict(unsigned int time, Location location, int user);

	void UpdateCAT();
	
	
	// The oposite of the above
	void SanitizePaths();

	void countPossibleConflicts();
	
	void CalculateCATCost();


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
	/*
		The Constraint avoidance table described on the paper.
	*/
	vector<Constraint> CAT;
	Conflict conflict;
	

	Conflicted deadlock; // Any deadlock detected will be stored here

	bool goal;

	// If this node has a valid solution, it must be set to true
	bool validNode;

	/*
		This element will cout how many consecutive swaps have this node done so far.
		This will be helpful to detect a corrdidors.
	*/
	int swapcounter;

	/*
		The main actor term will be used to describe the agent whos path is being modified 
		when expanding a node.
	*/

	int main_actor_id;

	// A cost used when two nodes have the same value, this cost breaks the ties
	int CATCost;
	
};