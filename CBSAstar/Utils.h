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
	DEADLOCK, BLOCKING_SIMPLE, BLOCKING_COMPLEX,
	BLOCKING_MULTIPLE
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
	A struct that represents the conflict element used in the CBS algorithm.
*/
struct Conflict{

	//Empty constructor, literally
	Conflict(){
		empty = true;
		replan_flag = false;
	}

	void clear(){
		users.clear();
		locations.clear();
		times.clear();
		empty = true;
		replan_flag = false;
	}

	std::vector<int> users;
	std::vector<Location> locations; // Fix for a multi location conflict
	std::vector<unsigned int> times;
	bool empty;
	
	bool replan_flag;
};



/*
	This struct represents 2 or more conflicted entities, used for
	the detection of special conflicts that Silver's algorithm is unable to solve.
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