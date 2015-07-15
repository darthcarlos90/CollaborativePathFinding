#include "Menu.h"

Menu::Menu(void){
	cout << "Welcome" << endl;
	PrintMainMenu();
}
Menu::~Menu(void){
	// Empty for now
}

void Menu::Execute(){
	while (type != 3){
		if (type == 1){
			RunTests();
			system("cls");
			PrintMainMenu();
		}
		else if (type == 2){
			LoadMap();
			system("cls");
			PrintMainMenu();
		}
	}
}

void Menu::RunTests(){
	system("pause");
	MAPF m(8, 8);
	PrintAlgorithmMenu();

	//Load the routes for the entities
	m.Start(algorithm_type);
	m.MoveEntities(algorithm_type);
	cout << "Finished" << endl;
	system("pause");
}

void Menu::LoadMap(){
	system("cls");
	string filename;
	int testcase;
	/*
		I know it looks ugly and I'm sorry.
	*/
	cout << "Please select the testcase you want to run: " << endl;
	cout << "1)  Simple testcase 1: 2 agents, several obstacles." << endl;
	cout << "2)  Simple testcase 2: 1 agent, several obstacles." << endl;
	cout << "3)  Simple testcase 3: 2 agents, small corridor." << endl;
	cout << "4)  Simple testcase 4: 2 agents, small map, no obstacles." << endl;
	cout << "5)  Simple testcase 5: 2 agents, big map, several obstacles." << endl;
	cout << "6)  Simple testcase 6: 3 agents, big map, several obstacles." << endl;
	cout << "7)  Simple testcase 7: 3 agents, bottleneck." << endl;
	cout << "8)  Blocking testcase 1: 2 agents, small bottleneck." << endl;
	cout << "9)  Blocking testcase 2: 2 agents, no obstacles." << endl;
	cout << "10) Complex testcase 1: 2 agents, Blocking + narrow corridor." << endl;
	cout << "11) Complex testcase 2: 2 agents, head to head." << endl;
	cout << "12) Complex testcase 3: 2 agents, narrow corridor + head to head collision." << endl;
	cout << "13) Complex testcase 4: 2 agents, blocking + bottleneck" << endl;
	cout << "14) Complex testcase 5: 3 agents, blocking + narrow corridor + head to head + small map." << endl;
	
	//getline(cin, filename);
	
	cin >> testcase;
	switch (testcase){
	case 1:
		filename = "testcase1.txt";
		break;
	case 2:
		filename = "testcase2.txt";
		break;
	case 3:
		filename = "testcase3.txt";
		break;
	case 4:
		filename = "testcase4.txt";
		break;
	case 5:
		filename = "testcaseBigMap.txt";
		break;
	case 6:
		filename = "testcaseBigMap3Units.txt";
		break;
	case 7:
		filename = "testcaseBottleneck.txt";
		break;
	case 8:
		filename = "testcaseBlocking2.txt";
		break;
	case 9:
		filename = "testcaseBlocking3.txt";
		break;
	case 10:
		filename = "testcaseBlockingDifficult.txt";
		break;
	case 11:
		filename = "testcaseHeadToHead.txt";
		break;
	case 12:
		filename = "testcaseNarrow.txt";
		break;
	case 13:
		filename = "testcaseSilversBlocking.txt";
		break;
	case 14:
		filename = "corridor.txt";
		break;
	}

	MAPF m(filename);
	//MAPF m("corridor.txt"); //Just for debugging Ill hardcode the filename
	if (m.isBroken()){
		cout << "There's been an error loading map.\n" << filename << " file doesn't exist.\nPlease check the name of the file and try again." << endl;
	}
	else {
		
		PrintAlgorithmMenu();
		
		//Load the routes for the entities
		m.Start(algorithm_type);
		m.MoveEntities(algorithm_type);
	}

	cout << "Finished" << endl;
	system("pause");
}

void Menu::PrintMainMenu(){
	cout << "Please select the type of tests you want to run: " << endl;
	cout << "(Write the number of the option you choose)" << endl;
	cout << "1) Algorithm Comparison Tests" << endl;
	cout << "2) Load a pre-generated testcase" << endl;
	cout << "3) Exit" << endl;

	cin >> type;
}

void Menu::PrintAlgorithmMenu(){
	cout << "Select the algorithm that you want to run:" << endl;
	cout << "1) Silver's algorithm." << endl;
	cout << "2) CBS algorithm." << endl;
	cout << "3) Hybrid algorithm." << endl;
	cin >> algorithm_type;
}