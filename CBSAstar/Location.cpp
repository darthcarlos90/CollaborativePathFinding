#include "Location.h"


Location::Location(int x, int y, int id){
	this->x = x;
	this->y = y;
	this->id = id;
}

Location::Location(int x, int y){
	this->x = x;
	this->y = y;
	id = -1; //no id, plain location
}

//Copy constructor
Location::Location(const Location& l){
	x = l.x;
	y = l.y;
	id = l.id;
}

Location::Location(){
	x = 0;
	y = 0;
	id = -1;
}

bool Location::operator == (const Location& l){
	return (this->x == l.x && this->y == l.y);
}

bool Location::operator!= (const Location& l){
	return (this->x != l.x && this->y != l.y);
}

bool Location::operator < (const Location &l){
	return (this->x < l.x && this->y < l.y);
}

bool Location::operator >(const Location& l){
	return(this->x > l.x && this->y > l.y);
}

bool Location::isAbove(Location l){
	return (this->x < l.x);
}

bool Location::isLeftOf(Location l){
	return (this->y > l.y);
}

Location& Location::operator= (const Location& l){
	x = l.x;
	y = l.y;
	id = l.id;

	return *this;
}