#include "Menu.h"

Menu::Menu(void){
	cout << "Welcome" << endl;
	//PrintMainMenu();
	type = 2;
}
Menu::~Menu(void){
	// Empty for now
}

void Menu::Execute(){
	//while (type != 3){
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
	//}
}

void Menu::RunTests(){
	
	system("pause");
}

void Menu::LoadMap(){
	system("cls");
	//string filename;
	//cin.clear();
	//cout << "Please write the name of the file to read from: " << endl;
	//getline(cin, filename);
	//MAPF m(filename);
	MAPF m("corridor.txt"); //Just for debugging Ill hardcode the filename
	if (m.isBroken()){
		cout << "There's been an error loading map.\nPlease check the name of the file and try again." << endl;
	}
	else {

		//Load the routes for the entities
		m.Start(2); //hardoced for now
		//m.RevisePaths(); //Revise the paths for any conflict, and solve it
		
		m.MoveEntities(2);
	}

	cout << "Finished" << endl;
	system("pause");
}

void Menu::PrintMainMenu(){
	cout << "Please select the type of tests you want to run: " << endl;
	cout << "(Write the number of theoption you choose)" << endl;
	cout << "1) Algorithm Comparison Tests" << endl;
	cout << "2) Load a map from a file" << endl;
	cout << "3) Exit" << endl;

	cin >> type;
}