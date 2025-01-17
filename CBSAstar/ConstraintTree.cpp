#include "ConstraintTree.h"

ConstraintTree::ConstraintTree(void){
	root = NULL;
	validSolution = false;
}

ConstraintTree::~ConstraintTree(void){
	terminate_tree(root);
}

/*
	Fix: Changed the return type to be a pointer
	Date: 15/05/2015
	Why?
	Because we need to modify the contents of the best node, not the contents of a copy
*/
CBTNode* ConstraintTree::getSolution(){
	if (root->hasChildren()){
		CBTNode* n = root;
		while (n->hasChildren()){
			n = n->getSmallestChild();
		}
		validSolution = n->isValidNode();
		return n;
	}
	//Otherwise the root is the actual solution
	else {
		validSolution = root->isValidNode();
		return root;
	}

}

void ConstraintTree::terminate_tree(CBTNode* n){
	if (n->hasChildren()){
		for (int i = 0; i < n->nChildren(); i++){
			terminate_tree(n->childAt(i));
		}
	}

	delete n;
	n = NULL;
}

bool ConstraintTree::HasValidSolution(){
	validSolution = getSolution()->isValidNode();
	return validSolution;
}

