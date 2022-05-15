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
	//Node st('1', 0, 0, 1, start);
	//Node n2('2', 1, 0, 2, any);
	//Node n3('3', 2, 0, 1, any);
	//Node n4('4', 0, 1, 1, any);
	//Node n5('5', 1, 1, 1, any);
	//Node gl('6', 2, 1, 1, goal);

	NodesVect.push_back(std::make_shared<Node>('1', 0, 0, 1, start));
	NodesVect.push_back(std::make_shared<Node>('2', 1, 0, 2, any));
	NodesVect.push_back(std::make_shared<Node>('3', 2, 0, 1, any));
	NodesVect.push_back(std::make_shared<Node>('4', 0, 1, 1, any));
	NodesVect.push_back(std::make_shared<Node>('5', 1, 1, 1, any));
	NodesVect.push_back(std::make_shared<Node>('6', 2, 1, 1, goal));


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
	//dummyNode n2('2', 1, 0, 1, start);
	//dummyNode n3('3', 2, 0, 5, any);
	//dummyNode n7('7', 3, 1, 1, any);

	newMap.push_back(std::make_shared<dummyNode>('2', 1, 0, 1, start));
	newMap.push_back(std::make_shared<dummyNode>('3', 2, 0, 5, any));
	newMap.push_back(std::make_shared<dummyNode>('7', 3, 1, 1, any));
}