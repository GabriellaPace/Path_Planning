#pragma once
#include "ClassNode.h"

using Sptr_toNode = std::shared_ptr<Node>;
using Wptr_toNode = std::shared_ptr<Node>;

std::vector<std::shared_ptr<Node>> NodesVect;	// vector of shared pointers to Nodes (declaration and definition)
std::vector<std::shared_ptr<dummyNode>> newMap;	// vector of shared pointers to dummyNodes
Wptr_toNode  ptrToStart = nullptr;
Wptr_toNode  ptrToGoal = nullptr;
float k_m = 0;

void ReadMap_firstTime() {
	NodesVect.push_back(std::make_shared<Node>('1', 0, 0, 10, start));
	NodesVect.push_back(std::make_shared<Node>('2', 1, 0, 20, any));
	NodesVect.push_back(std::make_shared<Node>('3', 2, 0, 10, any));
	NodesVect.push_back(std::make_shared<Node>('4', 0, 1, 10, any));
	NodesVect.push_back(std::make_shared<Node>('5', 1, 1, 10, any));
	NodesVect.push_back(std::make_shared<Node>('6', 2, 1, 10, goal));

	for (auto N : NodesVect) {
		if (N->nodeType == start) {
			ptrToStart = N;
		}
		else if (N->nodeType == goal) {
			ptrToGoal = N;
		}
	}
}


void ReadMap() {
	newMap.push_back(std::make_shared<dummyNode>('1', 0, 0, 10, any));	//ASSUMPTION: this fix always happen (so we don't heve 2 "start")
	newMap.push_back(std::make_shared<dummyNode>('2', 1, 0, 10, start));
	newMap.push_back(std::make_shared<dummyNode>('3', 2, 0, 50, any));
	newMap.push_back(std::make_shared<dummyNode>('7', 3, 1, 10, any));

	std::cout << " => RECEIVED NEW MAP\n";
}