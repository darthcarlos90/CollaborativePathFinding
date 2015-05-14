#pragma once
#include "Node.h"
#include "Map.h"
#include <algorithm>


class Agent{
public:
	Agent(Node location, Node destination, Map *m, int id, int D);
	~Agent(void);

	void SetLocationNode(Node n) { actualNode = n; }

	void SetDestination(Node n) { destination = n; }


	void setD(int val) { d = val; }
	int getD() { return d; }

	int getId() { return id; }

	void move(int time_to_move);
	void setTime(int time_to_set);

	/*
		Fix: All the Astar elements where moved here.
		Date: 30/04/2015
		Why?
		Because the Agent is the one that is "thinking", that's why it must
		be able to do all the calculations of Astar.
	*/

	void calculateRoute();// calculates the route using spatial astar
	//Execute a normal Astar using manhattan distance as heuristic
	void executeSpatialAstar(Node start, Node finish);

	//Execute spatial Astar until you find certain node
	void executeSpatialAstarUntilFound(Node start, Node toFind);

	void executeTimeSpaceAstar();

	void calculateRealHeuristic(Node* toCalculate, Node finish);

	
	int getSic();



	std::vector<Node> getSpatialRoute() { return spatial_route; }
	std::vector<Node> getPath() { return time_route; }

	

	/*
		Fix: This elements where brought from the map class.
		Date: 30/04/2015
		Why?
		Because the map is only in charge of giving you the adjacent elements, not on 
		making the calculations, those calculations are ment to be done by myself
		Note: This methods will remain public, if not used elsewhere, they will be changed
		to private.
	*/
	std::vector<Node> getAdjacents(Node element);
	std::vector<Node> getTimedAdjacents(Node element, int res_time);

	bool finished() { return !active; }

	

private:
	//Helper of the logit of time-space Astar
	void TimeSpaceAstarHelper(Node start, Node finish);
	//Making code mode readable, sort of ...
	bool AtClosedList(Node n);
	bool AtOpenList(Node n);
	bool AtTimedClosedList(Node n);
	bool AtTimedOpenList( Node n);
	void calculateSIC();


	Node actualNode;
	Node destination;

	Map* map;
	
	int d;

	bool active;
	bool reachedD;

	int id;

	int stepsTaken;
	bool replan;
	int tempD;

	int SIC; //Sum of Individual Costs


	int t;// as said, I will add t to every element to see who needs it and who doesnt

	std::vector<Node> spatial_openList;
	std::vector<Node> spatial_closedList;
	std::vector<Node> spatial_route;

	std::vector<Node> time_openList;
	std::vector<Node> time_closedList;
	std::vector<Node> time_route;
};