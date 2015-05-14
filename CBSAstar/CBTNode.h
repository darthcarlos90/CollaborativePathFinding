#include "Unit.h"
/*
	Class: CBTNode
	Description: This class represents the nodes inside a Constraint Tree.
	Author: Carlos
*/

class CBTNode{
public:
	CBTNode(void); // When you are creating a root node
	CBTNode(vector<Constraint> parent_constraints); // When you are creating a child node, it inherits its dads nodes
	CBTNode(vector<Constraint> parent_constraints, vector<Conflict> conflicts); // When expanding a node with more than one conflict, to mantain a binary tree form
	CBTNode(const CBTNode& n); //copy constructor
	~CBTNode(void);

	CBTNode* getLeft() { return left; }
	bool hasLeft();
	void setLeft(CBTNode* new_Left) { left = new_Left; }

	CBTNode* getRight() { return right; }
	bool hasRight();
	void setRight(CBTNode* new_right) { right = new_right; }

	void CalculatePaths(); // Calculates the paths of all the agents
	void ExpandNode(); //Expands the node when there is a found conflict

	bool isConsistent() { return consistent; }// all paths satisfy all constraints
	vector<Conflict> getConflicts() { return conflicts; }

	bool isValid() { return (conflicts.size() == 0); } // if it has conflicts
	

	int getCost() { return cost; }
	void calculateCost();

	CBTNode& operator = (const Node& n); // Asigment operator

	



private:
	CBTNode * left;
	CBTNode * right;
	int cost;
	
	vector<Unit*> units; // The units
	vector<vector<Node>> paths; // The paths
	/*
		Note: I wonder if it is ok to save only the units, or to save the whole paths as well.
	*/

	vector<Constraint> constraints;
	bool consistent;
	vector<Conflict> conflicts;
	
};