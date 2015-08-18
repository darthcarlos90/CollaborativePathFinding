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
	int executeSpatialAstarUntilFound(Location start, Node toFind);

	void executeTimeSpaceAstar(int starting_time);

	void executeTimeSpaceAstarFromLastIndex();

	void calculateRealHeuristic(Node* toCalculate, Location finish);

	int executebacksearchAstar(Location start, Location finish);

	
	int getSic();

	void ModifyRouteOnConstraints(vector<Constraint> constraints, bool reroute_flag, vector <Constraint> CAT); // Modifies the rute based on the constraints given

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
	std::vector<Node> getTimedAdjacents(Node element, int res_time, Location ending);
	std::vector<Node> getAdjacentsWithoutParents(Node element);
	std::vector<Node> getAdjacentsonConstraints(Node element, Location endint, vector<Constraint> constraints, int time);


	bool finished() { return !active; }

	int getX() const { return actualNode.getX(); }
	int getY() const { return actualNode.getY(); }

	Node getActualLocation() { return actualNode; }
	Location getLocation() { return actualNode.getLocation(); }
	Node getDestination() { return destination; }
	Location getDestinationLocation() { return destination.getLocation(); }
	Node getSartNode() { return startingPoint; }
	Location getStartLocation() { return startingPoint.getLocation(); }

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
	
	// TODO: Not used, eliminate soon
	Node GetSimpleEscapeNode(Location start);

	/*
		This method sets the stepsTaken to steps limit, so that the
		replanning is forced.
	*/
	void ActivateReplanFlag();

	
	bool NeedsPathVerification() { return needsPathVerification; }
	void SetPathVerificationFlag(bool val){ needsPathVerification = val; }
	void calculateSIC();
	// Clears the path of repeated useless elements at the end of the path
	void SanitizePath();

	// This method takes the agent back to it's starting position, and eliminates the path
	void resetElement();
	void setValidPath(bool val) { validSolution = val; }

	int getManhattanBetweenNodes();
	bool hasValidSolution() { return validSolution; }

	void setPriority(bool val) { priority = val; }
	bool getPriority() { return priority; }

	/*
		Debug methods that I will probably not use, or use only once.
		Or use only for debugging.
	*/
	bool debugNumberOfAdjacents() { return map->NumberAdjacents(destination.getLocation()) <= 2; }


	Location getLocationAtTime(int time){ return time_route[time].getLocation(); }
	

private:
	/*Helper functions*/
	void TimeSpaceAstarHelper(Location start, Location finish, int time);
	
	// Finder methods
	bool FindNodeAtList(Node n, vector<Node> list);
	bool FindNodeAtSpatialOpenList(Node n);
	bool FindNodeAtSpatialClosedList(Node n);
	bool FindNodeAtTimeOpenList(Node n);
	bool FindNodeAtTimeClosedList(Node n);
	int GetIndexOfElement(vector<Node> list, Node element);
	int GetIndexOfElementAtSpatialOpenList(Location element);
	int GetIndexOfElementAtSpatialClosedList(Location element);
	int GetIndexOfElementAtTimeOpenList(Location element);
	int GetIndexOfElementAtTimeClosedList(Location element);

	void reserveRoute(int starting_time); // To be used in the Silver's Astar
	void reserveRouteFromIndex(unsigned int index);
	void addToSpatialOpenList(Node n);
	void addToTimedSpatialOpenList(Node n);// Probably not used, still not gona remove it
	void addToTimeOpenList(Node n);
	void clearSpatialLists(bool clearSpatialRoute);
	void clearTimeLists(bool clearTimeRoute);
	
	void UpdateSpatialOpenList();
	void UpdateTimeOpenList();

	void AddToTimedSpatialOpenListCAT(Node n);
	
	void UpdateIndexSmallerSpatial();
	void UpdateIndexSmallerTime();

	void UpdateSpatialOpenListCAT();
	void UpdateIndexSmallerSpatialCAT();

	

	/*
		Helper function that allows us to find a special case:
		If the Astar has found the destination, but there are constraints for this agent after it has found its destination, lets check:
			- If those constraints are on the destination of this element
			- AND if one of those constraints is at t + 1
			- AND at that same timespace the adjacents of the destination are on constraints
			- Means that this agent must not yet reach the destination, so it must move to some other place.
		This special case is identified with this function.
		You might be asking yourself, why t + 1? The answer is simple:
		If you may recall on the method ModifyRouteOnConstraints, when the element is not able to wait on its last location due to a 
		constraint, it replans its last steps. If it is unable to find a route given the constraints, it goes back one step, and tries to 
		replan from there. But what happens if when we go back one step, we go to a step that is adjacent to the end. Well, on the next step
		it is going to find the goal node AND, therefore, break. Whast happens next is that all timespans after it found its destination, 
		this agent will be there even though it is not able to be there, leading us to an infinite tree expansion in CBS. If you find 
		this difficult to understand, you can always contact me and I try my best to explain it better :)
	*/
	bool FindSpecialCaseCBS(Location location, int t, vector<Constraint> constraints);


	/*
		This method runs a normal Astar algorithm, and stops until an element out
		of the critical zone is located, and returns that element.
	*/
	Node EscapeAstar(Location start); // The time at which a set of adjacent elements will be found
	
	bool ConstraintAstar(Location start, Location finish, int starting_time, vector<Constraint> constraints);
	
	bool validMovement(Location location, int time, vector<Constraint> constraints);

	int FindNumberOcurrancesCAT(Location location);


	
	
	int id;

	Node startingPoint;
	Node actualNode;
	Node destination;

	Map* map;
	
	unsigned int steps_limit;

	bool active;

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

	bool validSolution;

	vector<Constraint> CAT;

	bool priority;// TODO: Wipe this if this doesn't work

	int last_reserved;


};