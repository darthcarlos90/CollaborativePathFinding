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

	int x;
	int y;
	int id; //id of tthis location (id of the unit-to-be)
};

