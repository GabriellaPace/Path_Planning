#pragma once
#include "ClassNode.h"

std::vector<Node> Node::NodesList;

void ReadMap() {
	//NodesList.clear();
	Node n1('1', 0, 0, true, false);
	Node n2('2', 1, 0, false, false);
	Node n3('3', 2, 0, false, true);

	for (auto &N : Node::NodesList) {
		if (N.X == 0 && N.Y == 0) {
			N.key.first = 0.0f;
			N.key.second = 0.0f;
		}
		else if (N.X == 1 && N.Y == 0) {
			N.key.first = 1.0f;
			N.key.second = 4.0f;
		}
		else if (N.X == 2 && N.Y == 0) {
			N.key.first = 2.0f;
			N.key.second = 0.0f;
		}
	}
}