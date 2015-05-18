#pragma once
#include <hash_map>
#include <vector>
#include "Node.h"

class ReservationTable{
public:
	ReservationTable(void){};
	~ReservationTable(void){};

	void Reserve(int t, Node n, int userId);
	bool isReserved(Node element, int t);

private:
	std::hash_map<int, std::vector<Location>> reservations;
	
};