#pragma once
#include "FileManager.h"
#include "ConstraintTree.h"

/*
	Class MAPF
	Description: This class is in charge of all the pathfinding done by the agents. MAPF stands for Multiple Agent Path Finding.
	Author: Carlos Tirado
*/

class MAPF{
public:
	MAPF(string filename); // inputs information from a map (for static maps)
	MAPF(int size_x, int size_y, int max_players = 2); //inputs for random generation of map (for dynamic maps)
	~MAPF(void); //Destructor
	
	/*
		start the pathfinding, type represents what algorithm to use
		1 - Represents Silvers Algorithm for Pathfinding, and CBS for conflict resolution.
		2 - Represents CBS for pathfinding, and Silvers for conflict resolution
	*/
	void Start(int type);
	void MoveEntities(bool automatic); //The type thing is temporal
	//This method will revise the paths in look for any conflict
	void RevisePaths();

	bool isBroken() { return broken; }
	void printCosts(ostream& out);

	Matrix<int> getMatrix() { return *map->getData(); }
	vector<vector<Node>> getPaths() { return paths; }
	int numberPlayers() { return players.size(); }

	void resetEntities();
	void addRandomPlayer();
	void PrintPlayers(ostream& out);
	

private://
	void StartCBSPathFinding();
	void StartSilversPathFinding();
	void StartHybridPathFinding();

	void MoveBySilvers(bool hybrid, bool automatic);
	void MoveByCBS(bool automatic);
	

	bool existsInList(vector<int> list, int val);
	bool NodeExistsOnList(vector<Node> list, Node val);
	int getIndexOfAgent(int id);

	//Each method consists detects one type of conflict
	void Deadlock(); // returns the length of the critical path
	void SolveDeadLock(Conflicted c);
	void Blocking(); // Detects if a finished element blocks the path of another element
	void SolveBlockingSimple( Conflicted c); // Solves the blocking 
	void SolveBlockingComplex(Conflicted c);
	void DefaultHelper(Conflicted c);
	void SolveBlockingMultiple(Conflicted c);

	// Helper functions for the blocking element
	void SimpleBlocking();
	void MultipleBlocking();
	
	int countCriticalZone(Conflicted c, vector <Node>* criticalZoneNodes = NULL);
	int GetIndexAtArray(vector<Node> list, Location val); //This searches element by element, so first make sure the element is there

	void RunCBSUsingPlayers(vector<Agent> agents);

	//Method that solves the conflicts on the list using CBS
	void solveConflicts();

	//CBS helper method so that I don't have to repeat this code over and over again
	/*
		Fix: if RunCheck is true, then a check for certain events will be ran.
		Date:04/07/2015
		Why?
		Because it turned out that CBS can't deal with certain types of narrow path blocking
		situations, that is why the algorithm needs a bit of a help.
	*/
	void CBSHelper(bool RunCheck);

	// Helper function to detect blocking elements
	bool DetectBlockingHelper(unsigned int currentPlayer, unsigned int currentPath, Node destination, unsigned int* timeOc = NULL);

	bool AlreadyOnConflict(vector<int> agents, int type);
	bool IsSubset(vector<int> a, vector<int> b);

	// Helper functions to create a random map
	void AddMapObstacles(int limit);
	bool ValidMap();


	//TODO: Keep adding more elements throught the development of this project.
	FileManager* fr;
	Map* map;
	ConstraintTree* tree;
	vector<Agent> players;

	CBTNode* root; // The root node of the Tree

	bool broken;

	int time;

	vector<Conflicted> agent_conflicts;
	vector<vector<Node>> paths; // This is used to analyze the paths looking for conflicts

	int algorithm_type;
};