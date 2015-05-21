#pragma once
#include <hash_map>
#include <vector>
#include "Node.h"

class ReservationTable{
public:
	ReservationTable(void){};
	~ReservationTable(void){};

	void Reserve(int t, Node n, int userId);
	void addConstraint(Constraint c);
	bool isReserved(Node element, int t, int id); // an element is reserved by someone
	void addConstraints(std::vector<Constraint> constraints);
	void updateConstraint(int t, int userId, Node n);
	std::vector<Constraint> getConstraintList(int t);


private:
	std::hash_map<int, std::vector<Constraint>> reservations; // For an easier search
	
};