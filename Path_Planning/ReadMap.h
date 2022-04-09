#pragma once
#include "ClassNode.h"

std::vector<Node> Node::NodesList;

void ReadMap() {
	Node st('1', 0, 0, start);
	Node n2('2', 1, 0, any);
	Node n3('3', 2, 0, any);
	Node n4('4', 0, 1, any);
	Node n5('5', 1, 1, any);
	Node gl('6', 2, 1, goal);
}