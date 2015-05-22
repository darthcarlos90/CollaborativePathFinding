#include "FileReader.h"

FileReader::FileReader(std::string file, bool time_space){
	this->time_space = time_space;
	readData(file);
	
}

FileReader::~FileReader(void){
	/* This class is actually in charge of the data pointer, so
		this class must call the method delete. The rest of the classes that
		use this pointer, must only set it to null.
	*/
	/*
		Fix: The pointer is now deleted on the map class, not in this class.
		Date: 22/05/2015
		Why?
		Because we need to do this fix in order to make the sub map tha solves conflicts.
	*/
	//delete data;
	data = NULL;

}

/*
	Note to self: Because my matrix class works really lame, the
	coordinates are ..... changed ... to call it someway.
	Sorry :(
*/
void FileReader::readData(std::string filename){
	std::string reader; // Where we are going to save what we are reading
	std::ifstream myfile(filename); // retriever of the file

	int x = 0;
	int y = 0;

	int m_x = 0;
	int m_y = 0;

	if (myfile.is_open()){
		getline(myfile, reader);
		std::cout << reader << std::endl;
		string x_val;
		for (unsigned int i = 5; i < reader.size(); i++){
			x_val += reader[i];
		}
		x = atoi(x_val.c_str());
		rows = x;
		getline(myfile, reader);
		cout << reader << endl;
		string y_val;
		for (unsigned int i = 8; i < reader.size(); i++){
			y_val += reader[i];
		}
		y = atoi(y_val.c_str());
		columns = y;
		data = new Matrix<int>(x, y);
		if (time_space){
			getline(myfile, reader);
			cout << reader << endl;
			string n_players;
			for (unsigned int i = 9; i < reader.size(); i++){
				n_players += reader[i];
			}
			players = atoi(n_players.c_str());
			int id = 3;
			for (int i = 0; i < players; i++){
				getline(myfile, reader);
				int index = 0;
				string temp_x;
				string temp_y;
				for (int i = 0; i < reader.size(); i++){
					if (reader[i] == ' '){
						index = i + 1;
						break;
					}
					temp_x += reader[i];
				}

				for (int i = index; i < reader.size(); i++){
					if (reader[i] == ' '){
						index = i + 1;
						break;
					}
					temp_y += reader[i];
				}
				Location start(stoi(temp_x.c_str()),stoi(temp_y.c_str()), id);
				startings.push_back(start);
				temp_x = "";
				temp_y = "";
				for (int i = index; i < reader.size(); i++){
					if (reader[i] == ' '){
						index = i + 1;
						break;
					}
					temp_x += reader[i];
				}

				for (int i = index; i < reader.size(); i++){
					if (reader[i] == ' '){
						break;
					}
					temp_y += reader[i];
				}

				Location destination(stoi(temp_x.c_str()), stoi(temp_y.c_str()), id);
				endings.push_back(destination);
				id++;
			}

		}
		m_x = 0;

		while (getline(myfile, reader)){ //i dont know why it wont work other way
			m_y = 0;
			for (unsigned int i = 0; i < reader.size(); i++){
				if (reader[i] != ' '){
					data->set_element(m_x, m_y, reader[i] - '0');
					m_y++;
				}
			}
			m_x++;
		}

	}
}