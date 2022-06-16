#pragma once
#include "ClassNode.h"

using Wptr_toNode = std::shared_ptr<Node>;

std::vector<std::shared_ptr<Node>>		NodesVect;	// vector of shared pointers to Nodes (declaration and definition)
std::vector<std::shared_ptr<dummyNode>> newMap;		// vector of shared pointers to dummyNodes
Wptr_toNode  ptrToStart = nullptr,  ptrToGoal = nullptr;
float k_m = 0;
int map_count = 0;	// to change map in different iterations


#include "Image.h"

void readBMP() {
	Image img;
	img.Read("C:/Dev/Path_Planning/Maps/0_gradient.bmp");
	img.Export("C:/Dev/Path_Planning/Maps/0_gradient_COPY.bmp");
}
//gray = 0.2126*R + 0.7152*G + 0.0722*B;

//void writeBMP() {
//	const int width = 640;
//	const int height = 480;
//
//	Image image(width, height);
//
//	for (int y = 0; y < height; ++y) {	//rows
//		for (int x = 0; x < width; ++x) {	//coloumns
//			image.SetColor(Color((float)x / (float)width, 1.0f - ((float)x / (float)width), (float)y / (float)height), x, y);	//random colors
//		}
//	}
//
//	image.Export("C:/Dev/Path_Planning/Maps/image.bmp");
//}


void ReadMap() {
	newMap.clear();

	if (map_count < 2) {
		std::string path = "C:/Dev/Path_Planning/Maps/";
		//unsigned char* slope = readBMP( (path + std::to_string(map_count) + "_gradient.bmp").c_str() );
		
		//writeBMP();
		readBMP();
		//readBMP((path + std::to_string(map_count) + "_gradient.bmp").c_str());
		//C:/Dev/Path_Planning/Maps/0_gradient.bmp


		std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << map_count << "}\n";
		++map_count;
	}
}

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