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
	
	bool isEmpty();
	CBTNode getSolution();
	CBTNode* getRoot();
	



private:
	CBTNode * root;
	CBTNode* solution;
};