#pragma once
#include "Matrix.h"
#include "ReservationTable.h"
#include <algorithm>

class Map{
public:
	Map(Matrix<int>* mat);
	Map(unsigned int size_x, unsigned int size_y);
	Map(void);
	Map(const Map& m);
	~Map(void);

	int getRows() { return data->get_x_size(); }
	int getColumns() { return data->get_y_size(); }
	void setElement(int x, int y, int val);
	void setElement(Location l, int val);
	void printData();
	int getValueAt(int x, int y);
	int getValueAt(Location loc);
	

	//Another fix, this changes to public, and getAdjacents and get timed Adjacents go to unit
	vector<Node> adjacentHelper(Location element);//Adjacent helper function that returns adjacents
	int NumberAdjacents(Location element); // returns the number of adjacents of a location

	
	void reserve(int t, Node n, int id);
	bool isReserved(Node n, int t, int id);
	
	void cleanMap();
	void cleanObstacles();
	void cleanConstraintsReservations();

	/*
		Helper function that returns a suggested D for hte Silvers
		algorithm based on the size of the grid
	*/
	int CalculateD(); 

	//Returns a submap made up of the specifications given on the parameters
	Matrix<int> getSubData(int lowerX, int lowerY, int upperX, int upperY); 
	
	void setData(Matrix <int> val);
	Matrix<int>* getData(){ return data; }

	std::vector<Constraint> GetReservationTableConstraints();

	/*
		This element creates a sub-map based on the parameters given.
		Params:
			- Node blocking: The Node where the blocking element is located.
			- Node escape: The escape node where the blocking element is moving.
			- Node blocked: The element that is being blocked, the acces to
			- Node difference: The difference should help transform from normal map
				coordinates to submap coordinates.
	*/
	Map createSubMap(Location blocking, Location escape, Location blocked, Location* difference = NULL);

	bool hasData() { return has_data; }

	int getXValue() { return data->get_x_size(); }
	int getYValue() { return data->get_y_size(); }


private:

	Node getNode(int x, int y);

	Matrix<int> *data;
	//Matrix<Node> *nodes;//I dont know if I need this or not, so Ill just let it here
	ReservationTable reservationTable;

	int t; // this represents the map at a certain time

	bool has_data;  // Boolean for all those ugly pointer exceptions that may occur

};