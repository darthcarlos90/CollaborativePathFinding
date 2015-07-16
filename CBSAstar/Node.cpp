#include "Node.h"
#include <iostream>


Node::Node(void){
	//type -1 means empty node
	type = -1;
	h = -1;
	g = -1;
	x = -1;
	y = -1;
	f = -1;
	parent = NULL;
	has_parent = false;
	depth = 0;
}

Node::Node(int type, int tempg, int x_pos, int y_pos){
	this->type = type;
	this->tempg = tempg;
	this->x = x_pos;
	this->y = y_pos;
	g = 0;
	h = 0;
	f = g + h;
	parent = NULL;
	has_parent = false;
	my_location.id = type;
	my_location.x = x_pos;
	my_location.y = y_pos;
	depth = 0;
}

Node::Node(int type, Location location){
	this->type = type;
	tempg = 0;
	x = location.x;
	y = location.y;
	g = 0;
	h = 0;
	f = 0;
	parent = NULL;
	has_parent = false;
	my_location = location;
	my_location.id = type;
	depth = 0;
}

Node::~Node(void){
	if (parent && has_parent){ //just in case
		delete parent;
	}
	parent = NULL;
}

Node::Node(const Node& n){
	g = n.getG();
	tempg = n.getIndividualG();
	h = n.getH();
	f = g + h;
	type = n.getType();
	x = n.getX();
	y = n.getY();
	if (n.hasParent()){
		has_parent = false;
		this->setParent(n.getParent());
	}
	else { 
		parent = NULL; 
		this->has_parent = false;
	}
	this->my_location = n.getLocation();
	this->depth = n.depth;
}

Node& Node::operator= (const Node& n){
	g = n.getG();
	tempg = n.getIndividualG();
	h = n.getH();
	f = g + h;
	type = n.getType();
	x = n.getX();
	y = n.getY();
	if (n.hasParent()){
		has_parent = false;
		//we are setting a new parent, so we need to set it false to allocate new memory
		this->setParent(n.getParent());
	}
	else {
		parent = NULL;
		this->has_parent = false;
	}
	this->my_location = n.getLocation();
	this->depth = n.depth;

	return *this;
}


void Node::calculateManhattanHeuristic(Location destination){

	int finalX = abs(x - destination.x);
	int finalY = abs(y - destination.y);
	//We are now blocking diagonal movements
	/*if (finalX < finalY){
		h = (finalX * 14);
	}
	else {
		h = finalY * 14;
	}

	h += (abs(finalX - finalY) * 10);*/
	h = (finalX + finalY) * 10;
}


bool Node::operator < (const Node & n)const{
	if (f == n.getF()){
		if (h == n.getH()){
			return (g < n.getG());
		}
		return (h < n.getH());
	}
	return (f < n.getF());
}

bool Node::operator >(const Node &n)const{
	if (f == n.getF()){
		if (h == n.getH()){
			return (g > n.getG());
		}
		return (h > n.getH());
	}
	return (f > n.getF());
}

bool Node::operator==(const Node &n)const{
	return (this->x == n.getX() && this->y == n.getY());
}

bool Node::operator!=(const Node& n)const{
	return !(this->x == n.getX() && this->y == n.getY());
}

void Node::setParent(Node value){
	if (!has_parent){
		parent = new Node();
	}
	*parent = value;
	has_parent = true;
	depth += value.getDepth() + 1;
}

bool Node::hasParent() const{
	return has_parent;
}

//This is a method of the node, because it needs to acces its parents
void Node::calculateG(){
	//This method will recursively go to each parent and calculate its g
	g = tempg;
	if (has_parent){
		parent->calculateG();
		g += parent->getG();
	}
}

void Node::clearParent(){
	delete parent;
	parent = NULL;
	has_parent = false;
	depth = 0;
}

unsigned int Node::getDepth() const{ 
	if(has_parent) return depth; // Security measure
	else return 0;
}

void Node::ConvertToSubmapCoordinates(Location exchange){
	x = x - exchange.x;
	y = y - exchange.y;
	my_location.x = x;
	my_location.y = y;
}

void Node::ConvertToMapCoordinates(Location exchange){
	x = x + exchange.x;
	y = y + exchange.y;
}

void Node::printValue(){
	std::cout << "(" << x << ", " << y << ")" << std::endl;
}