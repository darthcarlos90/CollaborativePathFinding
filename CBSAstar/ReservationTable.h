#pragma once
#include <hash_map>
#include <vector>
#include "Node.h"


struct Location{
	Location(int new_x, int new_y){
		x = new_x;
		y = new_y;
	}
	int x;
	int y;
};

class ReservationTable{
public:
	ReservationTable(void){};
	~ReservationTable(void){};

	void Reserve(int t, Node n, int userId);
	bool isReserved(Node element, int t);

private:
	std::hash_map<int, std::vector<Location>> reservations;
	
};