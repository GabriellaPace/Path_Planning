#pragma once
#include "ClassNode.h"

//std::list<Node*> NodesList;

void ReadMap() {
	//NodesList.clear();

	Node n1('1', 0, 0, true, false);
	//NodesList.push_front(&n1);

	Node n2('2', 1, 0, false, false);
	//NodesList.push_front(&n2);

	Node n3('3', 2, 0, false, true);
	//NodesList.push_front(&n3);

	n1.key.first = 0.0f;
	n1.key.second = 0.0f;

	n2.key.first = 1.0f;
	n2.key.second = 4.0f;

	n3.key.first = 2.0f;
	n3.key.second = 0.0f;
	//n3.key.first = 1.0f;   //works anyway
	//n3.key.second = 4.0f;
}