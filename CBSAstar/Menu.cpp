#include "Menu.h"

Menu::Menu(void){
	cout << "Welcome" << endl;
	PrintMainMenu();
}
Menu::~Menu(void){
	// Empty for now
	if (fileManager){
		delete fileManager;
		fileManager = NULL;
	}
}

void Menu::Execute(){
	while (type != 4){
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
		else if (type == 3){
			LoadManualMap();
			system("cls");
			PrintMainMenu();
		}
	}
}

void Menu::RunTests(){
	fileManager = new FileManager("test.txt");
	const clock_t total_time = clock();
	for (int i = 2; i <= 13; i++){
		fileManager->myfile << "Testcase: " << i - 1 << endl;
		MAPF m(8, 8, i);
		//PrintAlgorithmMenu();
		// Run same test for different algorithms
		fileManager->myfile << "Map: " << endl;
		fileManager->myfile << m.getMatrix() << endl;
		fileManager->myfile << "Number of players " << m.numberPlayers() << endl;
		m.PrintPlayers(fileManager->myfile);
		for (int index = 1; index <= 3; index++){
			// TODO: Change so that after the paths have been calculated, go back to starting position and recalculate route.
			//Load the routes for the entities
			fileManager->myfile << "< ======================================================================== >" << endl;
			switch (index){
			case 1:
				fileManager->myfile << "Silver's Algorith Results:" << endl;
				break;
			case 2:
				fileManager->myfile << "CBS Results:" << endl;
				break;
			case 3:
				fileManager->myfile << "Hybrid results:" << endl;
			}
			const clock_t begin_time = clock();
			m.Start(index);
			float calculation_time = float(clock() - begin_time) / CLOCKS_PER_SEC;
			const clock_t progress_time = clock();
			m.MoveEntities(true);
			float moving_time = float(clock() - progress_time) / CLOCKS_PER_SEC;

			cout << "Finished" << endl;
			cout << "Saving information into file" << endl;

			m.PrintPaths(fileManager->myfile);
			
			fileManager->myfile << "Time taken to calculate paths: " << calculation_time << "s" << endl;
			fileManager->myfile << "Time taken to progress through the path " << moving_time << "s" << endl;
			fileManager->myfile << "Total time of execution for this map " << calculation_time + moving_time << "s" << endl;
			m.printCosts(fileManager->myfile);
			fileManager->myfile << endl;
			m.resetEntities();
			m.cleanReservationsConstraints();
		}
	}
	float total_running_time = float(clock() - total_time) / CLOCKS_PER_SEC;
	fileManager->myfile << "Total running time: " << total_running_time << endl;
	fileManager->closeFile();
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
	cout << "15) Unsolvable testcase" << endl;
	
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
	case 15:
		filename = "unsolvableTest.txt";
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
		m.MoveEntities(false);
	}

	cout << "Finished" << endl;
	system("pause");
}

void Menu::LoadManualMap(){
	//TODO: Code this for later use
}

void Menu::PrintMainMenu(){
	cout << "Please select the type of tests you want to run: " << endl;
	cout << "(Write the number of the option you choose)" << endl;
	cout << "1) Algorithm Comparison Tests" << endl;
	cout << "2) Load a pre-generated testcase" << endl;
	cout << "3) Load manually a testcase file" << endl;
	cout << "4) Exit" << endl;

	cin >> type;
}

void Menu::PrintAlgorithmMenu(){
	cout << "Select the algorithm that you want to run:" << endl;
	cout << "1) Silver's algorithm." << endl;
	cout << "2) CBS algorithm." << endl;
	cout << "3) Hybrid algorithm." << endl;
	cin >> algorithm_type;
}