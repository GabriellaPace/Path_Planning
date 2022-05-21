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
	if (count == 0) {
		newMap.push_back(std::make_shared<dummyNode>('1', 0, 0, 10, start));
		newMap.push_back(std::make_shared<dummyNode>('2', 1, 0, 20, any));
		newMap.push_back(std::make_shared<dummyNode>('3', 2, 0, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('4', 0, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('5', 1, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('6', 2, 1, 10, goal));

		++count;
	}
	else if (count == 1) {
		newMap.push_back(std::make_shared<dummyNode>('1', 0, 0, 10, any));	//ASSUMPTION: this fix always happen (so we don't heve 2 "start")
		newMap.push_back(std::make_shared<dummyNode>('2', 1, 0, 10, start));
		newMap.push_back(std::make_shared<dummyNode>('3', 2, 0, 50, any));
		newMap.push_back(std::make_shared<dummyNode>('7', 3, 1, 10, any));

		++count;
	}
	else if (count == 2) {
		++count;
	}

	std::cout << " => RECEIVED NEW MAP\n";
}