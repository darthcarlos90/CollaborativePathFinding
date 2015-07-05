/*
	Filename: Utils
	Description: This Header file is created in order to store the various structs that will be created in one single place
	Author: Carlos Tirado
*/

#pragma once
#include "Location.h"
#include <vector>

/*
	This enum represents the types of conflicts that the Silvers algorithm
	could't solve.
*/

enum conflict_type{
	DEADLOCK, BLOCKING_SIMPLE, BLOCKING_COMPLEX
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
	unsigned int t; // The time when it is reserved
};


/*
	A struct that represents the conflict element
*/
struct Conflict{
	Conflict(Location v, int t){
		this->v = v;
		this->t = t;
		empty = false;
		destination_conflict = false;
	}

	//Empty constructor, literally
	Conflict(){
		t = -1;
		empty = true;
		destination_conflict = false;
	}

	void addUser(int id){
		users.push_back(id);
	}

	std::vector<int> users;
	Location v;
	unsigned int t;
	bool empty;
	bool destination_conflict;
};

/*
	This struct represents 2 or more conflicted entities 
*/

struct Conflicted{
	
	
	std::vector<int> agents; //The ids of the agents involved in the conflict detected
	int type; //The type of the conflict;
	std::vector<Location> locations; // At what position where the elements when they found the conflict
	std::vector<int> times; // At what time in each of the elements the incident occured
	
	/*
		Fix: The map is not needed
		Date: 25/05/2015
		Why?
		Because there is no need to acces the map by the CBS Algorithm since the algorithm only makes th enetities wait, 
		not reroute.
	*/
	//Map m; // A peace of the map where the conflict will be solved;

};