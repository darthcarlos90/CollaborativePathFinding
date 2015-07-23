#include "Map.h"

Map::Map(Matrix<int>* mat){
	if (mat){
		data = mat;
		//findNodes(); //Only for use in spatial Astar, not this new method :D
		cout << *data << endl;
		has_data = true;
	}
	else {
		has_data = false;
	}
}

Map::Map(unsigned int size_x, unsigned int size_y){
	data = new Matrix<int>(size_x, size_y);
	has_data = true;
}

//Empty constructor for several purposes
Map::Map(void){
	data = NULL; //Make the data pointer null
	has_data = false;
}

//Copy cosntructor
Map::Map(const Map& m){
	data = new Matrix<int>();
	*data = *m.data; // trying to create a copy of the data, not point to the pointer
	reservationTable = m.reservationTable;
	t = m.t;
	has_data = m.has_data;
}

Map::~Map(void){
	delete data;
	data = NULL; 
	//delete nodes;
}

/*
	Helper method that retrieves the adjacent elements of a Node, if an adjacent element is a
	wall, it will be ignored
*/
vector<Node> Map::adjacentHelper(Location element){
	vector<Node> result;
	int type;

	//x & y elements of the element in consideration
	int x = element.x;
	int y = element.y;
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

	std::sort(result.begin(), result.end());

	return result;

}


void Map::setElement(int x, int y, int val){
	data->set_element(x, y, val);
}

void Map::setElement(Location l, int val){
	data->set_element(l.x, l.y, val);
}

void Map::printData(){
	cout << *data << endl;
}

void Map::reserve(int t, Node n, int id){
	reservationTable.Reserve(t, n, id);
	reservationTable.Reserve(t + 1, n, id); // All reservations must include a reservation for next t
}

bool Map::isReserved(Node n, int t, int id){
	return reservationTable.isReserved(n, t, id);
}

int Map::getValueAt(int x, int y){
	return data->get_element(x, y);
}

int Map::getValueAt(Location loc){
	return data->get_element(loc.x, loc.y);
}

void Map::cleanMap(){
	for (int i = 0; i < data->get_x_size(); i++){
		for (int j = 0; j < data->get_y_size(); j++){
			if (data->get_element(i, j) != 1){
				data->set_element(i, j, 0);
			}
		}
	}
}

// Set all the obstacle elements to 0
void Map::cleanObstacles(){
	for (int i = 0; i < data->get_x_size(); i++){
		for (int j = 0; j < data->get_y_size(); j++){
			if (data->get_element(i, j) == 1){
				data->set_element(i, j, 0);
			}
		}
	}
}

// TODO: Improve this
int Map::CalculateD(){
	return ((data->get_x_size() * data->get_y_size()) / 4);
}

Matrix<int> Map::getSubData(int lowerX, int lowerY, int upperX, int upperY){
	// We add 1 to make it row/column inclusive
	int x_size = (upperX - lowerX) + 1;
	int y_size = (upperY - lowerY) + 1;
	
	Matrix<int> matrix(x_size, y_size);
	int x = 0;
	int y = 0;
	// Populate the matrix with the values from this matrix
	// Mark our matrix with -1 creating a "danger zone"
	for (int i = lowerX; i <= upperX; i++){
		for (int j = lowerY; j <= upperY; j++){
			if (data->get_element(i, j) != 1){
				matrix.set_element(x, y, 0);
				data->set_element(i, j, -1); // Set the data to -1 so that we can later acces it
			}
			else matrix.set_element(x, y, 1);
			
			y++;
		}
		x++;
		y = 0;
	}
	//Uncomment for debugging
	/*cout << *data;
	cout << matrix;*/
	
	return matrix;

}

void Map::setData(Matrix<int> val) { 
	if (!has_data) data = new Matrix<int>();
	*data = val;
	has_data = true;
}

std::vector<Constraint> Map::GetReservationTableConstraints(){
	return reservationTable.getFullConstraints();
}

Map Map::createSubMap(Location blocking, Location escape, Location blocked, Location* difference){
	
	/*
	Get the row where the blocking element is.
	*/
	Location lowerBounds = blocking;
	Location upperBounds = blocking;

	/*
		If the escape element is on the same row as the blocking element, then
		grab the upper and lower rows.
	*/
	if (blocking.y == escape.y){
		if(lowerBounds.y > 0) 
			lowerBounds.y = lowerBounds.y - 1;
		if(upperBounds.y < data->get_y_size() - 1)
			upperBounds.y = upperBounds.y + 1;
	} //Otherwise, get the row where the escape element is
	else {
		if (lowerBounds.y > escape.y) 
			lowerBounds.y = escape.y;
		else if (upperBounds.y < escape.y) 
			upperBounds.y = escape.y;
	}

	/*
		Get the elements that are on the furthest side of the 
	*/
	if (lowerBounds.x > escape.x) 
		lowerBounds.x = escape.x;
	if (upperBounds.x < escape.x) 
		upperBounds.x = escape.x;
	if (lowerBounds.x > blocked.x) 
		lowerBounds.x = blocked.x;
	if (upperBounds.x < blocked.x) 
		upperBounds.x = blocked.x;
	
	//Finally, add the elements if they are outside our danger zone
	if (lowerBounds.y > blocked.y) lowerBounds.y = blocked.y;
	if (upperBounds.y < blocked.y) upperBounds.y = blocked.y;

	/*
		There are some situations, where leaving the exact size of the submap may 
		cause deadlocks to happen. Since CBS apparently is not optimized for
		deadlocks, the submap will be expanded a bit.
	*/

	if(upperBounds.x < data->get_x_size() - 1)upperBounds.x = upperBounds.x + 1;
	if(upperBounds.y < data->get_y_size() - 1)upperBounds.y = upperBounds.y + 1;
	if(lowerBounds.x > 0) lowerBounds.x = lowerBounds.x - 1;
	if(lowerBounds.y > 0)lowerBounds.y = lowerBounds.y - 1;

	//Now that we have the bounds of our submap, we can create the matrix with the data
	Matrix<int> *subdata = new Matrix<int>();
	*subdata = getSubData(lowerBounds.x, lowerBounds.y, upperBounds.x, upperBounds.y);

	//Now we can return a map based on the subdata
	Map submap(subdata);

	subdata = NULL;

	// If there is a pointer, lets feed it data
	if (difference){
		*difference = lowerBounds;
	}

	//return the result
	return submap;
}

void Map::cleanConstraintsReservations(){
	reservationTable.clean();
}