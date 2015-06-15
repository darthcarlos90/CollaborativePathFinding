/*
	Fix: I need a location class now, since there are some more operations I need the class to do,
	and a simple struct wont sufice for this.
	Date: 15/06/2015
	Why?
	Just explained it ...
*/
#pragma once



class Location{
public:
	
	Location(int x, int y, int id);
	Location(int x, int y);
	Location();

	bool isAbove(Location l);
	bool isLeftOf(Location l);

	bool operator == (const Location& l);
	bool operator < (const Location& l);
	bool operator > (const Location& l);

// I will still leave this elements as public elements
	int x;
	int y;
	int id; //id of this location (id of the unit-to-be)


private:
	//Just in case I need some private elements
};