#pragma once
#include "ClassNode.h"

using Sptr_toNode = std::shared_ptr<Node>;
using Wptr_toNode = std::shared_ptr<Node>;

std::vector<std::shared_ptr<Node>> NodesVect;	// vector of shared pointers to Nodes (declaration and definition)
std::vector<std::shared_ptr<dummyNode>> newMap;	// vector of shared pointers to dummyNodes
Wptr_toNode  ptrToStart = nullptr;
Wptr_toNode  ptrToGoal = nullptr;
float k_m = 0;
int count = 0;	// to change map in different iterations


void ReadMap() {
	newMap.clear();

	if (count == 0) {			//ok
		newMap.push_back(std::make_shared<dummyNode>('0', 0, 0, 10, start));
		newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 20, any));
		newMap.push_back(std::make_shared<dummyNode>('2', 2, 0, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('3', 0, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('6', 2, 1, 10, goal));
		++count;
	}
	else if (count == 1) {		//ok
		newMap.push_back(std::make_shared<dummyNode>('5', 1, 1, 60, any));
		
		//newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 5, any));
		//newMap.push_back(std::make_shared<dummyNode>('6', 2, 1, 2, goal));
		//reducing only the cost of (1,0) doesn't have the same effect, because the cost keeps being the max beetween two nodes
		++count;
	}
	else if (count == 2) {
		newMap.push_back(std::make_shared<dummyNode>('0', 0, 0, 10, any));	//ASSUMPTION: this fix always happen (so we don't heve 2 "start")
		//newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 20, start));
		newMap.push_back(std::make_shared<dummyNode>('5', 1, 1, 60, start));
		newMap.push_back(std::make_shared<dummyNode>('7', 3, 0, 10, any));
		++count;
	}
	//else if (count == 3) {												// it doesn't support goal change!
	//	newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 10, any));
	//	newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, start));
	//	newMap.push_back(std::make_shared<dummyNode>('6', 3, 0, 10, goal));
	//	++count;
	//}
	else if (count == 3) {		//ok
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, start)); //and goal  ->  ptrToStart == ptrToGoal

		++count;
	}
	else {
		return;
	}

	std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << count-1 << "}\n";
}