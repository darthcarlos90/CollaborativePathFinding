#include "ReservationTable.h"

void ReservationTable::Reserve(int t, Node n, int userId){
	//Reservation r(userId, n);
	Location l(n.getX(), n.getY());
	//Find if there are any reservations at time t
	if (reservations.find(t) == reservations.end()){
		//not found
		std::vector<Location> reserv;
		
		reserv.push_back(l);
		reservations.insert(std::make_pair(t, reserv));
	}
	else {
		//found
		reservations[t].push_back(l);
	}
}

//Look for a node to see if it is reserved at time t
bool ReservationTable::isReserved(Node element, int t){
	bool reserved = false;
	std::vector<Location> temp = reservations[t];
	for (unsigned int i = 0; i < temp.size(); i++){
		if (temp[i].x == element.getX() && temp[i].y == element.getY()){
			reserved = true;
			break;
		}
	}
	return reserved;
}