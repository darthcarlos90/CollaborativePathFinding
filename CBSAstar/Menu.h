#pragma once

#include "MAPF.h"

class Menu{
public:
	Menu(void);
	~Menu(void);

	void Execute();

private:
	// Private Methos
	void RunTests();
	void LoadMap();
	void PrintMainMenu();


	// private variables
	int type;
};