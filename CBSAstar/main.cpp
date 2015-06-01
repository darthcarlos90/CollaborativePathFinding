#include "MAPF.h"


int main(void){
	/*string filename;
	cout << "Welcome" << endl;
	cout << "Please write the name of the file to read from: " << endl;
	getline(cin, filename);
	MAPF m(filename);*/
	MAPF m("testcaseNarrow.txt"); //Just for debugging Ill hardcode the filename
	//Load the routes for the entities
	m.Start(1); //hardoced for now
	m.RevisePaths(); //Revise the paths for any conflict, and solve it
	m.MoveEntities(1);
	cout << "Finished" << endl;
	system("pause");
	//TODO: When an agent is close to reach the end, it restarts it's search. Check the flags and the lists.

	return 0;
}