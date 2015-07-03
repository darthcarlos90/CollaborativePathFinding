#pragma once
#include "FileReader.h"
#include "ConstraintTree.h"

/*
	Class MAPF
	Description: This class is in charge of all the pathfinding done by the agents. MAPF stands for Multiple Agent Path Finding.
	Author: Carlos Tirado
*/

class MAPF{
public:
	MAPF(string filename); // inputs information from a map (for static maps)
	MAPF(int size_x, int size_y); //inputs for random generation of map (for dynamic maps)
	~MAPF(void); //Destructor
	
	/*
		start the pathfinding, type represents what algorithm to use
		1 - Represents Silvers Algorithm for Pathfinding, and CBS for conflict resolution.
		2 - Represents CBS for pathfinding, and Silvers for conflict resolution
	*/
	void Start(int type);
	void MoveEntities(int type); //The type thing is temporal
	//This method will revise the paths in look for any conflict
	void RevisePaths();
	

private:
	void StartCBSPathFinding();
	void StartSilversPathFinding();

	void MoveBySilvers();
	void MoveByCBS();

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

	bool DeadLockHelper(vector<Node> path1, vector<Node> path2, vector<Location>* locations = NULL, vector<int>* times = NULL);

	
	int countCriticalZone(Conflicted c, vector <Node>* criticalZoneNodes = NULL);
	int GetIndexAtArray(vector<Node> list, Node val); //This searches element by element, so first make sure the element is there

	void RunCBSUsingPlayers(vector<Agent> agents);

	//Method that solves the conflicts on the list using CBS
	void solveConflicts();

	//CBS helper method so that I don't have to repeat this code over and over again
	void CBSHelper();

	//TODO: Keep adding more elements throught the development of this project.
	FileReader* fr;
	Map* map;
	ConstraintTree* tree;
	vector<Agent> players;

	CBTNode* root; // The root node of the Tree

	bool broken;

	int time;

	vector<Conflicted> agent_conflicts;
	vector<vector<Node>> paths; // This is used to analyze the paths looking for conflicts
};