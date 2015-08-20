#include "Menu.h"

#define TOTAL_TESTCASES 2

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
	system("cls");
	cout << "Select a type of tests to run: " << endl;
	cout << "1) Tests maps with no obstacles." << endl;
	cout << "2) Tests maps with obstacles." << endl;
	int option;
	cin >> option;
	switch (option){
	case 1:
		RunObstacleLessTests();
		break;
	case 2:
		RunObstacleTests();
		break;
	default:
		cout << "Wrong option, select again." << endl;
		RunTests();
		break;
	}
	
}

void Menu::RunObstacleTests(){
	fileManager = new FileManager("ObstacleTests.txt");
	const clock_t total_time = clock();
	for (int i = 2; i <= 13; i++){
		fileManager->myfile << "Testcase: " << i - 1 << endl;
		MAPF m(8, 8,true, i);
		// Run same test for different algorithms
		fileManager->myfile << "Map: " << endl;
		fileManager->myfile << m.getMatrix() << endl;
		fileManager->myfile << "Number of players " << m.numberPlayers() << endl;
		m.PrintPlayers(fileManager->myfile);
		for (int index = 1; index <= 3; index++){
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

void Menu::RunObstacleLessTests(){
	fileManager = new FileManager("NoObstaclesResults.txt");
	vector<float> averageCostSilver;
	vector<float> averageCostCBS;
	vector<float> averageCostHybrid;
	vector<float> averageTimeSilvers;
	vector<float> averageTimeCBS;
	vector<float> averageTimeHybrid;
	vector<int> validSolutionsSilvers;
	vector<int> validSolutionsCBS;
	vector<int> validSolutionsHybrid;
	const clock_t total_time = clock();
	for (int i = 2; i <= 13; i++){
		int silverWinns = 0;
		int CBSwins = 0;
		int hybridWins = 0;
		float sumSilvers = 0.0f;
		float sumCBS = 0.0f;
		float sumHyb = 0.0f;
		int validSilv = 0;
		int validCBS = 0;
		int validHyb = 0;
		for (int testcases = 1; testcases <= TOTAL_TESTCASES; testcases++){
			fileManager->myfile << "Testcase: " << testcases << endl;
			MAPF m(8, 8, false, i);
			// Run same test for different algorithms
			fileManager->myfile << "Map: " << endl;
			fileManager->myfile << m.getMatrix() << endl;
			fileManager->myfile << "Number of players " << m.numberPlayers() << endl;
			m.PrintPlayers(fileManager->myfile);
			for (int index = 1; index <= 3; index++){
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
					break;
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
				

				switch (index){
				case 1:
					if (m.valid()) validSilv++;
					sumSilvers += calculation_time;
					m.printCosts(fileManager->myfile,&silverWinns );
					break;
				case 2:
					if (m.valid()) validCBS++;
					sumCBS += calculation_time;
					m.printCosts(fileManager->myfile, &CBSwins);
					break;
				case 3:
					if (m.valid()) validHyb++;
					sumHyb += calculation_time;
					m.printCosts(fileManager->myfile, &hybridWins);
					break;
				}

				fileManager->myfile << endl;
				m.resetEntities();
				m.cleanReservationsConstraints();
			}

		}

		averageTimeSilvers.push_back(sumSilvers / (float)TOTAL_TESTCASES);
		averageTimeCBS.push_back(sumCBS / (float)TOTAL_TESTCASES);
		averageTimeHybrid.push_back(sumHyb / (float)TOTAL_TESTCASES);
		averageCostSilver.push_back((float)silverWinns / (float)TOTAL_TESTCASES);
		averageCostCBS.push_back((float)CBSwins / (float)TOTAL_TESTCASES);
		averageCostHybrid.push_back((float)hybridWins / (float)TOTAL_TESTCASES);
		validSolutionsSilvers.push_back(((float)validSilv * 100.0f) / (float)TOTAL_TESTCASES);
		validSolutionsCBS.push_back(((float)validCBS * 100.0f) / (float)TOTAL_TESTCASES);
		validSolutionsHybrid.push_back(((float)validHyb * 100.0f) / (float)TOTAL_TESTCASES);
		
	}
	float total_running_time = float(clock() - total_time) / CLOCKS_PER_SEC;
	fileManager->myfile << "Total running time: " << total_running_time << endl << endl;
	fileManager->myfile << "Silvers statistics: " << endl;
	
	for (unsigned int i = 0; i < averageTimeSilvers.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, average time " << averageTimeSilvers[i] << endl;
	}
	fileManager->myfile << endl;
	for (unsigned int i = 0; i < averageCostSilver.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, average cost " << averageCostSilver[i] << endl;
	}
	fileManager->myfile << endl;
	for (unsigned int i = 0; i < validSolutionsSilvers.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, percentage of succes " << validSolutionsSilvers[i] << "%" <<endl;
	}
	fileManager->myfile << endl;
	fileManager->myfile << "CBS statistics: " << endl;
	for (unsigned int i = 0; i < averageTimeCBS.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, average time " << averageTimeCBS[i] << endl;
	}
	fileManager->myfile << endl;
	for (unsigned int i = 0; i < averageCostCBS.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, average cost " << averageCostCBS[i] << endl;
	}
	fileManager->myfile << endl;
	for (unsigned int i = 0; i < validSolutionsCBS.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, percentage of succes " << validSolutionsCBS[i] << "%" << endl;
	}
	fileManager->myfile << endl;
	fileManager->myfile << "Hybrid statistics: " << endl;
	for (unsigned int i = 0; i < averageTimeHybrid.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, average time " << averageTimeHybrid[i] << endl;
	}
	fileManager->myfile << endl;
	for (unsigned int i = 0; i < averageCostHybrid.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, average cost " << averageCostHybrid[i] << endl;
	}
	fileManager->myfile << endl;
	for (unsigned int i = 0; i < validSolutionsHybrid.size(); i++){
		fileManager->myfile << "For " << i + 2 << " agents, percentage of succes " << validSolutionsHybrid[i] << "%" << endl;
	}
	
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
	cout << "15) Unsolvable testcase." << endl;
	cout << "16) Random generated testcase." << endl;
	cout << "17) No obstacles generated debug testcase, 3 agents." << endl;
	cout << "18) Small box, 3 agents." << endl;
	
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
	case 16:
		filename = "random1.txt";
		break;
	case 17:
		filename = "testcasetest.txt";
		break;
	case 18:
		filename = "smthg.txt";
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