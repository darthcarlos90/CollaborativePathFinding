#include "Unit.h"
/*
	Class: CBTNode
	Description: This class represents the nodes inside a Constraint Tree.
	Author: Carlos
*/

class CBTNode{
public:
	CBTNode(void);
	~CBTNode(void);



	int getCost() { return cost; }
	void calculateCost();



private:
	int cost;
	//TODO: LEFT HERE, IN THE DESIGN OF THE NODES AND THE TREE , USE AS EXAMPLE THE BINARY TREE YOU CREATED A LITTLE WHILE AGO :)
	
};