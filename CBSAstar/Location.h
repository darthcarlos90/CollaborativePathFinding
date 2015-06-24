/*
	Fix: I need a location class now, since there are some more operations I need the class to do,
	and a simple struct wont sufice for this.
	Date: 15/06/2015
	Why?
	At the begining, the node class was used for all kinds of representation of a location, but as the project
	grew in complexity, so did the complexity of the Node class, giving a hard time when passing it as 
	paramenters to methods, when the only thing needed was a location (i.e when passing a node as a paramenter, and
	having to copy all of the parents of the node, even thought they where never used). That is why the location
	was modified to be a class, so it can have its own assigment operators, copy constructor, etc, and be used
	where the other complex elements of a node are not necesary.
*/
#pragma once



class Location{
public:
	
	Location(int x, int y, int id);
	Location(int x, int y);
	Location(const Location& l); // Copy constructor
	Location();

	bool isAbove(Location l);
	bool isLeftOf(Location l);

	bool operator == (const Location& l);
	bool operator < (const Location& l);
	bool operator > (const Location& l);
	Location& operator = (const Location& l);

// I will still leave this elements as public elements
	int x;
	int y;
	int id; //id of this location (id of the unit-to-be)


private:
	//Just in case I need some private elements
};