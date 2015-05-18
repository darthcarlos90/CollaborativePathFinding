/*
	Filename: Utils
	Description: This Header file is created in order to store the various structs that will be created in one single place
	Author: Carlos Tirado
*/

#pragma once
#include <vector>

/*
Struct created to easily group the locations being read.
*/
struct Location{
	Location(int x, int y, int id){
		this->x = x;
		this->y = y;
		this->id = id;
	};

	Location(int x, int y){
		this->x = x;
		this->y = y;
		id = -1; //no id, plain location
	}

	Location(){
		x = 0;
		y = 0;
		id = -1;
	}

	bool operator == (const Location& l){
		return (this->x == l.x && this->y == l.y);
	}

	int x;
	int y;
	int id; //id of this location (id of the unit-to-be)
};

/*
Struct created to represent the Constraints and its elements
*/
struct Constraint{
	Constraint(int id, Location location, int t){
		this->id = id;
		this->location = location;
		this->t = t;
	}

	bool operator == (const Constraint& c){
		return (this->id == c.id 
			&& this->location == c.location 
			&& this->t == c.t);
	}

	int id; // The id of the node in this constraint
	Location location; // The location on th grid that is reserved
	int t; // The time when it is reserved
};


/*
	A struct that represents the conflict element
*/
struct Conflict{
	Conflict(Location v, int t){
		this->v = v;
		this->t = t;
		empty = false;
	}

	//Empty constructor, literally
	Conflict(){
		t = -1;
		empty = true;
	}

	void addUser(int id){
		users.push_back(id);
	}

	std::vector<int> users;
	Location v;
	int t;
	bool empty;
};