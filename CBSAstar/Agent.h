#pragma once
#include "Node.h"
#include "Map.h"
#include <algorithm>


class Agent{
public:
	Agent(Node location, Node destination, Map *m, int id, unsigned int D);
	~Agent(void);

	//Copy constructor
	//Agent(const Agent& other);

	void SetLocationNode(Node n) { actualNode = n; }

	void SetDestination(Node n) { destination = n; }


	void setD(int val) { steps_limit = val; }
	int getD() { return steps_limit; }

	int getId() { return id; }
	void setId(int new_id) { id = new_id; }

	// Moves to the element on the time t
	void move(unsigned int t);

	/*
		This function moves the entity to the spot their suposed to be on the time t
	*/
	void moveEntity(unsigned int t);

	//void setTime(int time_to_set);

	/*
		Fix: All the Astar elements where moved here.
		Date: 30/04/2015
		Why?
		Because the Agent is the one that is "thinking", that's why it must
		be able to do all the calculations of Astar.
	*/

	void calculateRoute();// calculates the route using spatial astar
	//Execute a normal Astar using manhattan distance as heuristic
	void executeSpatialAstar(Location start, Location finish);

	//Execute spatial Astar until you find certain node
	void executeSpatialAstarUntilFound(Location start, Node toFind);

	void executeTimeSpaceAstar(int starting_time);

	void executeTimeSpaceAstarFromLastIndex();

	void calculateRealHeuristic(Node* toCalculate, Location finish);

	
	int getSic();

	void ModifyRouteOnConstraints(vector<Constraint> constraints); // Modifies the rute based on the constraints given

	std::vector<Node> getSpatialRoute() { return spatial_route; }
	std::vector<Node> getPath() { return time_route; }

	//Used to solve the bug fix where both entities stop and advance
	void setPath(std::vector<Node> new_path) { time_route = new_path; }
	

	/*
		Fix: This elements where brought from the map class.
		Date: 30/04/2015
		Why?
		Because the map is only in charge of giving you the adjacent elements, not on 
		making the calculations, those calculations are ment to be done by myself
		Note: This methods will remain public, if not used elsewhere, they will be changed
		to private.
	*/
	std::vector<Node> getAdjacents(Node element, Location ending);
	std::vector<Node> getTimedAdjacentsWithoutParents(Node location, int time);
	std::vector<Node> getTimedAdjacents(Node element, int res_time);
	std::vector<Node> getAdjacentsWithoutParents(Node element);

	bool finished() { return !active; }

	int getX() const { return actualNode.getX(); }
	int getY() const { return actualNode.getY(); }

	Node getActualLocation() { return actualNode; }
	Location getLocation() { return actualNode.getLocation(); }
	Node getDestination() { return destination; }
	Location getDestinationLocation() { return destination.getLocation(); }

	bool isOnMyRoute(Node n);
	void AddNodeToPathAtTimeT(Node n, unsigned int t);
	void ReroutePathUsingSpatialAstar(int time);
	void modifyMap(vector <Node> otherPath);
	/*
		The parameters:
		keepRoute: True if you want a completely new route, false if you only want to update
		the route you already have.
		time: the current time t to search for adjacents
	*/
	void MoveToClosestEscapeElement(bool keepRoute, Location start);

	void ReroutePathUsingCBS();

	// Repeats the value at the index, the number of times indicated in the parameter
	void RepeatStepAtIndex(unsigned int index, int times);

	void PushElementAtTheBackOfRoute(Node val);
	unsigned int pathSize() { return time_route.size(); }

	Node GetEscapeNodeNotOnRoute(Location start, vector<Node> path, bool lowerThan);

	/*
		This method sets the stepsTaken to steps limit, so that the
		replanning is forced.
	*/
	void ActivateReplanFlag();

	Node getPartialDestination() { return partialDestination; }
	void setPartialDestination(Node val);
	bool hasPartialDestination() { return has_partial_destination; }
	
	bool NeedsPathVerification() { return needsPathVerification; }
	void SetPathVerificationFlag(bool val){ needsPathVerification = val; }


private:
	//Helper functions
	void TimeSpaceAstarHelper(Location start, Location finish, int time);
	bool FindNodeAtList(Node n, vector<Node> list); //Find a node at a given list
	void calculateSIC();
	void reserveRoute(int starting_time); // To be used in the Silver's Astar
	void reserveRouteFromIndex(unsigned int index);
	void addToSpatialOpenList(Node n);
	void addToTimeOpenList(Node n);
	int GetIndexOfElement(vector<Node> list, Node element);
	void clearSpatialLists(bool clearSpatialRoute);
	
	Node GetSmallestNodeFromSpatialOpenList();
	Node GetSmallestNodeFromTimeOpenList();
	
	void UpdateIndexSmallerSpatial();
	void UpdateIndexSmallerTime();

	/*
		This method runs a normal Astar algorithm, and stops until an element out
		of the critical zone is located, and returns that element.
	*/
	Node EscapeAstar(Location start); // The time at which a set of adjacent elements will be found
	/*
		This method will look for the closest element that is not part of the path given on the
		parameters.
	*/
	
	
	int id;

	Node actualNode;
	Node destination;
	Node partialDestination;

	Map* map;
	
	unsigned int steps_limit;

	bool active;
	bool has_partial_destination;

	unsigned int stepsTaken;

	int SIC; //Sum of Individual Costs
	bool needsPathVerification;

	std::vector<Node> spatial_openList;
	std::vector<Node> spatial_closedList;
	std::vector<Node> spatial_route;
	unsigned int index_lower_spatial_openList;

	std::vector<Node> time_openList;
	std::vector<Node> time_closedList;
	std::vector<Node> time_route;
	unsigned int index_lower_time_openList;

	std::vector<Node> partial_path_nodes;
};