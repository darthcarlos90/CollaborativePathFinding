#pragma once
#include "Matrix.h"
#include "ReservationTable.h"





class Map{
public:
	Map(Matrix<int>* mat);
	~Map(void);
	int getRows() { return data->get_x_size(); }
	int getColumns() { return data->get_y_size(); }
	void setElement(int x, int y, int val);
	void printData();
	int getValueAt(int x, int y);
	void setValue(int x, int y, int val);

	//Another fix, this changes to public, and getAdjacents and get timed Adjacents go to unit
	vector<Node> adjacentHelper(Node element);//Adjacent helper function that returns adjacents

	
	void reserve(int t, Node n, int id);
	bool isReserved(Node n, int t, int id);
	void cleanMap();

	/*
		Helper function that returns a suggested D for hte Silvers
		algorithm based on the size of the grid
	*/
	int CalculateD(); 


private:

	Node getNode(int x, int y);

	Matrix<int> *data;
	//Matrix<Node> *nodes;//I dont know if I need this or not, so Ill just let it here
	ReservationTable reservationTable;

	int t; // this represents the map at a certain time

};