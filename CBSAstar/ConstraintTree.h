/*
	Class: Open Tree
	Description: This class represents the tree Conflict Tree.
	Author: Carlos Tirado
*/
#include "CBTNode.h"



class ConstraintTree{
public:
	ConstraintTree(void);
	~ConstraintTree();
	
	void insertRoot(CBTNode* n) { root = n; }
	CBTNode* getRoot() { return root; }
	CBTNode getSolution(); // Look for the solution, which will always be the leftmost node



private:
	void terminate_tree(CBTNode* n);
	CBTNode * root;
	
};