#include "MAPF.h"


int main(void){
	/*string filename;
	cout << "Welcome" << endl;
	cout << "Please write the name of the file to read from: " << endl;
	getline(cin, filename);
	MAPF m(filename);*/
	MAPF m("file.txt"); //Just for debugging Ill hardcode the filename
	//Load the routes for the entities
	m.Start();
	m.MoveEntities();
	cout << "Finished" << endl;
	system("pause");


	return 0;
}