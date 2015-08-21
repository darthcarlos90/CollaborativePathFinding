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

	// Debug method to test a broken pointer
	void clearData();
	

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
			- Vector of Locations: This is a vector with the locations of all the locations
				involved in the conflict.
			- Node difference: The difference should help transform from normal map
				coordinates to submap coordinates.
	*/
	Map createSubMap(vector<Location> locations, Location* difference = NULL, Location *newlowerBounds = NULL, Location *newUpperBounds = NULL);
	Matrix<int> CreateSubData(vector<Location> locations, Location* difference = NULL, Location *newlowerBounds = NULL, Location *newUpperBounds = NULL);

	// If the solution couldnt be found with the map size, make it bigger
	Matrix<int> expandMap(int old_x, int old_y, Location pastDifference, Location * difference = NULL, Location *newlowerBounds = NULL, Location *newUpperBounds = NULL);


	bool hasData() { return has_data; }

	int getXValue() { return data->get_x_size(); }
	int getYValue() { return data->get_y_size(); }
	int getNumberObstacles() { return numberObstacles; }
	int getNumberSpaces() { return numberSpaces; }


private:

	Node getNode(int x, int y);
	void countStuff();

	Matrix<int> *data;
	//Matrix<Node> *nodes;//I dont know if I need this or not, so Ill just let it here
	ReservationTable reservationTable;

	int t; // this represents the map at a certain time

	bool has_data;  // Boolean for all those ugly pointer exceptions that may occur
	int numberObstacles;
	int numberSpaces;

};