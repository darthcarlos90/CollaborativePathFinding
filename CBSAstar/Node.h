#pragma once
#include <cmath>
#include <vector>
#include "Utils.h"


class Node{
public:
	Node(void);
	Node(int type, int tempg, int x_pos, int y_pos);
	Node(const Node& n);//copy constructor
	~Node(void);

	void calculateF() { f = g + h; }
	int getF() const { return f; }

	void setIndividualG(int val) { tempg = val; }
	int getIndividualG() const{ return tempg; }
	void calculateG();
	void setG(int val) { g = val; }
	int getG() const { return g; }

	void calculateManhattanHeuristic(Node destination);
	

	void setH(int val) { h = val; }
	int getH() const { return h; }

	void setType(int val) { type = val; }
	int getType() const { return type; }

	void setLocation(int x, int y) {
		this->x = x;
		this->y = y;
	}
	
	int getX() const { return x; }
	int getY() const { return y; }

	bool operator < (const Node& n);
	bool operator == (const Node& n);
	bool operator != (const Node& n);
	Node& operator = (const Node& n);

	void setParent(Node value);
	bool hasParent() const;
	Node getParent() const{ return *parent; }

	void clearParent(); //Method used so that the parent is forgotten

	Location getLocation() const { return my_location; }


private:
	int f;
	int x;
	int y;
	int g;
	int h;
	int tempg;
	
	
	int type;
	
	bool has_parent;

	Node* parent;
	
	Location my_location;
};