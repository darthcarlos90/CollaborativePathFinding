/*
	Filename: Utils
	Description: This Header file is created in order to store the various structs that will be created in one single place
	Author: Carlos Tirado
*/

#pragma once

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
	int id; //id of tthis location (id of the unit-to-be)
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
	}

	void addUser(int id){
		users.push_back(id);
	}

	void addUser(Agent u){
		users.push_back(u.getId());
	}

	vector<int> users;
	Location v;
	int t;
};