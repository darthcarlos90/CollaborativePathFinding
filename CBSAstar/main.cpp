#include "MAPF.h"


int main(void){
	string filename;
	cout << "Welcome" << endl;
	cout << "Please write the name of the file to read from: " << endl;
	getline(cin, filename);
	MAPF m(filename);
	//MAPF m("testcaseBlocking2.txt"); //Just for debugging Ill hardcode the filename
	if (m.isBroken()){
		cout << "There's been an error loading map.\nPlease check the name of the file and try again." << endl;
	}
	else {
		
		//Load the routes for the entities
		m.Start(1); //hardoced for now
		m.RevisePaths(); //Revise the paths for any conflict, and solve it
		m.MoveEntities(1);
	}
	
	cout << "Finished" << endl;
	system("pause");

	return 0;
}