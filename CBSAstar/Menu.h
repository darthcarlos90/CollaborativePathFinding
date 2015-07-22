#pragma once

#include "MAPF.h"
#include <time.h>

class Menu{
public:
	Menu(void);
	~Menu(void);

	void Execute();

private:
	// Private Methos
	void RunTests();
	void LoadMap();
	void LoadManualMap();
	void PrintMainMenu();
	void PrintAlgorithmMenu();


	// private variables
	int type; 
	int algorithm_type;

	// File Administration
	FileManager *fileManager;
};