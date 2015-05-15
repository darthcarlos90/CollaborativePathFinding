#include "ConstraintTree.h"

ConstraintTree::ConstraintTree(void){
	root = NULL;
}

ConstraintTree::~ConstraintTree(void){
	terminate_tree(root);
}

CBTNode ConstraintTree::getSolution(){
	if (root->hasChildren()){
		CBTNode n = *root;
		while (n.hasChildren()){
			n = *n.getSmallestChild();
		}

		return n;
	}
	//Otherwise the root is the actual solution
	else {
		return *root;
	}

}

void ConstraintTree::terminate_tree(CBTNode* n){
	if (n->hasChildren()){
		for (unsigned int i = 0; i < n->nChildren(); i++){
			terminate_tree(n->childAt(i));
		}
	}

	delete n;
	n = NULL;
}

