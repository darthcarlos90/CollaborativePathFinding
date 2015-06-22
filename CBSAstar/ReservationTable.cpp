#include "ReservationTable.h"

void ReservationTable::Reserve(int t, Node n, int userId){
	
	//Create the new constraint
	Constraint c(userId, n.getLocation(), t);
	
	//Find if there are any reservations at time t
	if (reservations.find(t) == reservations.end()){
		//not found
		//Create a vector of constraints
		std::vector<Constraint> reserv;
		//Add the newly created constraint
		reserv.push_back(c);
		// Inser the pair
		reservations.insert(std::make_pair(t, reserv));
	}
	else {
		//found
		reservations[t].push_back(c);
	}

	constraints.push_back(c);
}

void ReservationTable::addConstraint(Constraint c){

	//Find if there are any reservations at time t
	if (reservations.find(c.t) == reservations.end()){
		//not found
		//Create a vector of constraints
		std::vector<Constraint> reserv;
		//Add the newly created constraint
		reserv.push_back(c);
		// Inser the pair
		reservations.insert(std::make_pair(c.t, reserv));
	}
	else {
		//found
		reservations[c.t].push_back(c);
	}

	constraints.push_back(c);
}

//Look for a node to see if it is reserved at time t by someone different to id
bool ReservationTable::isReserved(Node element, int t, int id){
	bool reserved = false;
	if (reservations.find(t) != reservations.end()){
		//Get the constraints at time t
		std::vector<Constraint> temp = reservations[t];

		for (unsigned int i = 0; i < temp.size(); i++){
			//If the current constraint has the same location as the input
			if (temp[i].location == element.getLocation()){
				if (temp[i].id != id){ // it is reserved by someone else
					reserved = true;
					break;
				}
			}
		}
	}
	
	return reserved;
}

void ReservationTable::addConstraints(std::vector<Constraint> constraints){
	for (unsigned int i = 0; i < constraints.size(); i++){
		addConstraint(constraints[i]);
	}
}

//Update the location of a reservation at a time t, this is used when an agent changes its path
void ReservationTable::updateConstraint(int t, int userId, Node n){
	for (unsigned int i = 0; i < reservations[t].size(); i++){
		if (reservations[t][i].id == userId){
			reservations[t][i].location = n.getLocation();
			break;
		}
	}
}

//Get the constraints at a time t
std::vector<Constraint> ReservationTable::getConstraintList(int t){
	return reservations[t];
}