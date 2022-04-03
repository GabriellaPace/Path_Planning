#pragma once
#include "ClassNode.h"

void ReadMap() {
	Node n1('1', 0, 0, true, false);
	Node n2('2', 1, 0, false, false);
	Node n3('3', 2, 0, false, true);

	n1.key.first = 0.0f;
	n1.key.second = 0.0f;

	n2.key.first = 1.0f;
	n2.key.second = 4.0f;

	n3.key.first = 2.0f;
	n3.key.second = 0.0f;
	//n3.key.first = 1.0f;   //works anyway
	//n3.key.second = 4.0f;
}