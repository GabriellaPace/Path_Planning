#pragma once
#include "ClassNode.h"


void ReadMap_firstTime() {
	Node st('1', 0, 0, 1, start);
	Node n2('2', 1, 0, 2, any);
	Node n3('3', 2, 0, 1, any);
	Node n4('4', 0, 1, 1, any);
	Node n5('5', 1, 1, 1, any);
	Node gl('6', 2, 1, 1, goal);

	//new Node('1', 0, 0, 1, start);
}


void ReadMap() {
	dummyNode n2('2', 1, 0, 1, start);
	dummyNode n3('3', 2, 0, 5, any);
	dummyNode n7('7', 3, 1, 1, any);
}