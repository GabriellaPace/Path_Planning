#pragma once
#include "ClassNode.h"
#include "Image.h"

using Sptr_toNode = std::shared_ptr<Node>;

std::vector<std::shared_ptr<Node>>		NodesVect;	// vector of shared pointers to Nodes (declaration and definition)
std::vector<std::shared_ptr<dummyNode>> newMap;		// vector of shared pointers to dummyNodes

int map_count = 0;	// to change map in different iterations
std::string img_path = "C:/Dev/Path_Planning/Maps/";


std::shared_ptr<dummyNode> findDummyptr(int xx, int yy) {    // find the pointer of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx;
	auto it = find_if(newMap.begin(), newMap.end(),
		[&x, &y](const std::shared_ptr<dummyNode>& obj) {return ((*obj).X == x && (*obj).Y == y); });
	if (it != newMap.end()) {
		idx = (int)std::distance(newMap.begin(), it);
		return newMap[idx];
	}
	else {
		return nullptr;
	}
}


//gray = 0.2126*R + 0.7152*G + 0.0722*B;
void Image::ReadMap_bmp(const char * path)
{
	std::ifstream f;
	f.open(path, std::ios::in | std::ios::binary);	//reading (::in) a file in binary

	if (!f.is_open()) { // = if the opening was NOT successful
		std::cout << "ERROR: File could not be opened.\n";
		return;
	}

	const int fileHeaderSize = 14;	//fixed
	const int informationHeaderSize = 40;	//fixed

	// reinterpret_cast:  doesn't modify the data in any way, just interprets it in a different way
	// static_cast:       keeps the value and changes the type
	unsigned char fileHeader[fileHeaderSize];
	f.read(reinterpret_cast<char*>(fileHeader), fileHeaderSize);

	if (fileHeader[0] != 'B' || fileHeader[1] != 'M') {		//check that the file we are reading is actually a bitmap:
		std::cout << "ERROR: The specified path does not correspond to a bitmap image.\n";
		f.close();
		return;
	}

	unsigned char informationHeader[informationHeaderSize];
	f.read(reinterpret_cast<char*>(informationHeader), informationHeaderSize);

	int fileSize = fileHeader[2] + (fileHeader[3] << 8) + (fileHeader[4] << 16) + (fileHeader[5] << 24);	//joining bytes
	m_width = informationHeader[4] + (informationHeader[5] << 8) + (informationHeader[6] << 16) + (informationHeader[7] << 24);
	m_height = informationHeader[8] + (informationHeader[9] << 8) + (informationHeader[10] << 16) + (informationHeader[11] << 24);

	m_colors.resize(m_width * m_height);

	const int paddingAmount = ((4 - (m_width * 3) % 4) % 4);	//calculating how much padding there is

	for (int y = 0; y < m_height; ++y) {	//rows
		for (int x = 0; x < m_width; ++x) {	//coloumns
			unsigned char color[3];

			f.read(reinterpret_cast<char*>(color), 3);	//reading the colors of 1 pixel (need 3B)

			//in .bmp the order is (B,G,R), so we start from last one
			m_colors[y*m_width + x].R = static_cast<float>(color[2]); // / 255.0f;
			m_colors[y*m_width + x].G = static_cast<float>(color[1]); // / 255.0f;
			m_colors[y*m_width + x].B = static_cast<float>(color[0]); // / 255.0f;

			uint8_t grey = static_cast<uint8_t>(m_colors[y*m_width + x].R);

			//std::cout << "X: " << x << ", Y: " << y << ", c: " << m_colors[y*m_width + x].R * 255.0f << std::endl << std::endl;
			newMap.push_back(std::make_shared<dummyNode>(x, y, grey, any));
		}

		f.ignore(paddingAmount);
	}

	f.close();
	std::cout << "File read.\n";

	return;
}


void ReadMap() {
	newMap.clear();

	if (map_count < 1) {
		//readBMP((path + std::to_string(map_count) + "_gradient.bmp").c_str());
		Image img;
		img.ReadMap_bmp((img_path + std::to_string(map_count) + "_gradient.bmp").c_str());

		std::shared_ptr<dummyNode> dummy;
		//int x_in, y_in;
		//std::cout << "Insert coordinates of START node: x y\n";
		//std::cin >> x_in; std::cin >> y_in;
		//dummy = findDummyptr(x_in, y_in);
		dummy = findDummyptr(0, 0);	
		dummy->nodeType = start;
		//std::cout << "Insert coordinates of GOAL node: x y\n";
		//std::cin >> x_in; std::cin >> y_in;
		//dummy = findDummyptr(x_in, y_in);
		dummy = findDummyptr(15, 15);	
		dummy->nodeType = goal;

		std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << map_count << "}\n";
		//++map_count;
	}
	else {
		return;
	}
}

//Manual Node insertion (ReadMap())
/*
void ReadMap() {
	newMap.clear();

	if (map_count == 0) {
		newMap.push_back(std::make_shared<dummyNode>('0', 0, 0, 10, start));
		newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 20, any));
		newMap.push_back(std::make_shared<dummyNode>('2', 2, 0, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('3', 0, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, goal));
		++map_count;
	}
	else if (map_count == 1) {
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 60, any));
		
		//newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 5, any));
		//newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 2, goal));
		//reducing only the cost of (1,0) doesn't have the same effect, because the cost keeps being the max beetween two nodes
		++map_count;
	}
	else if (map_count == 2) {
		newMap.push_back(std::make_shared<dummyNode>('0', 0, 0, 10, any));	//ASSUMPTION: this fix always happen (so we don't heve 2 "start")
		//newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 20, start));
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 60, start));
		newMap.push_back(std::make_shared<dummyNode>('6', 3, 0, 10, any));
		++map_count;
	}
	else if (map_count == 3) {	//goal change
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('2', 2, 0, 50, any));
		newMap.push_back(std::make_shared<dummyNode>('6', 3, 0, 10, goal));
		++map_count;
	}
	else if (map_count == 4) {
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 60, any));
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, start)); //and goal  ->  ptrToStart == ptrToGoal  ->  ARRIVED
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, goal));	 //and goal  ->  ptrToStart == ptrToGoal  ->  ARRIVED

		++map_count;
	}
	else {
		return;
	}

	std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << map_count-1 << "}\n";
}
*/