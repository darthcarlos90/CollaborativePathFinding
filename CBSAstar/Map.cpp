#include "Map.h"

Map::Map(Matrix<int>* mat){
	data = mat;
	//findNodes(); //Only for use in spatial Astar, not this new method :D
	cout << *data << endl;
}

Map::~Map(void){
	data = NULL; //I'm not in charge of destroying this pointer
	//delete nodes;
}

/*
	Helper method that retrieves the adjacent elements of a Node, if an adjacent element is a
	wall, it will be ignored
*/
vector<Node> Map::adjacentHelper(Node element){
	vector<Node> result;
	int type;

	//x & y elements of the element in consideration
	int x = element.getX();
	int y = element.getY();
	/*
		Fix: The diagonal movements are disbaled.
		Date: 21/05/2015
		Why?
		Because there are certain scenarios where two elements should collision, but because there is a 
		diagonal movement, they dont.
		TODO: Find a way to avoid collision on diagonal movemens.
		NOTE: For now, the diagonal movements are commented.
	*/
	//pretty much self explanatory, if not, then I'll try to explain in on a coment ... later ... yeah ..
	/*if (x - 1 >= 0 && y - 1 >= 0){
		type = data->get_element(x - 1, y - 1);
		if (type != 1){
			result.push_back(Node(type, 14, x - 1, y - 1));
		}
	}*/

	if (y - 1 >= 0){
		type = data->get_element(x, y - 1);
		if (type != 1){
			result.push_back(Node(type, 10, x, y - 1));
		}
	}

	/*if (x + 1 < data->get_x_size() && y - 1 >= 0){
		type = data->get_element(x + 1, y - 1);
		if (type != 1){
			result.push_back(Node(type, 14, x + 1, y - 1));
		}
	}*/

	if (x - 1 >= 0){
		type = data->get_element(x - 1, y);
		if (type != 1){
			result.push_back(Node(type, 10, x - 1, y));
		}
	}

	if (x + 1 < data->get_x_size()){
		type = data->get_element(x + 1, y);
		if (type != 1){
			result.push_back(Node(type, 10, x + 1, y));
		}
	}

	/*if (y + 1 < data->get_y_size() && x - 1 >= 0){
		type = data->get_element(x - 1, y + 1);
		if (type != 1){
			result.push_back(Node(type, 14, x - 1, y + 1));
		}
	}*/

	if (y + 1 < data->get_y_size()){
		type = data->get_element(x, y + 1);
		if (type != 1){
			result.push_back(Node(type, 10, x, y + 1));
		}
	}

	/*if (x + 1 < data->get_x_size() && y + 1 < data->get_y_size()){
		type = data->get_element(x + 1, y + 1);
		if (type != 1){
			result.push_back(Node(type, 14, x + 1, y + 1));
		}
	}*/

	return result;

}


void Map::setElement(int x, int y, int val){
	data->set_element(x, y, val);
}

void Map::printData(){
	cout << *data << endl;
}

void Map::reserve(int t, Node n, int id){
	reservationTable.Reserve(t, n, id);
	//reservationTable.Reserve(t + 1, n, id); // Al reservations must include a reservation for next t
}

bool Map::isReserved(Node n, int t, int id){
	return reservationTable.isReserved(n, t, id);
}

int Map::getValueAt(int x, int y){
	return data->get_element(x, y);
}

void Map::setValue(int x, int y, int val){
	data->set_element(x, y, val);
}

void Map::cleanMap(){
	for (int i = 0; i < data->get_x_size(); i++){
		for (int j = 0; j < data->get_y_size(); j++){
			if (data->get_element(i, j) > 1){
				data->set_element(i, j, 0);
			}
		}
	}
}

int Map::CalculateD(){
	return ((data->get_x_size() * data->get_y_size()) / 4);
}