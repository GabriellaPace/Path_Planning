#pragma once
#include "ClassNode.h"

//std::vector<std::weak_ptr<Node>> Node::NodesVect;
std::vector<std::shared_ptr<Node>> Node::NodesVect;


void ReadMap() {
	Node st('1', 0, 0, start);
	//Node::NodesVect.push_back(std::make_shared<Node>(st));

	Node n2('2', 1, 0, any);
	//Node::NodesVect.push_back(std::make_shared<Node>(n2));

	Node n3('3', 2, 0, any);
	//Node::NodesVect.push_back(std::make_shared<Node>(n3));

	Node n4('4', 0, 1, any);
	//Node::NodesVect.push_back(std::make_shared<Node>(n4));

	Node n5('5', 1, 1, any);
	//Node::NodesVect.push_back(std::make_shared<Node>(n5));

	Node gl('6', 2, 1, goal);
	//Node::NodesVect.push_back(std::make_shared<Node>(gl));
}