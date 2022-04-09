#include "ReadMap.h"

using QT = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
//template<typename T>
void print_queue(QT q) {					// debug
	std::cout << "Queue:" << std::endl;
	Node tmp;
	while (!q.empty()) {
		tmp = q.top();
		tmp.print_NodeKey();
		q.pop();
	}
	std::cout << std::endl;
}

// definition of Node's static variables
float Node::k_m = 0.0f;
int Node::X_start = -1;
int Node::Y_start = -1;

int main() {
	//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;

	ReadMap();

	// function Initialize()
	for (auto &N : Node::NodesList) {
		if (N.nodeType == goal) {
			N.calculateKey();
			queue.push(N);
		}
	}

	//for (auto &N : Node::NodesList) {
	//	if (N.X == 0 && N.Y == 0) {
	//		N.key.first = 0.0f;
	//		N.key.second = 0.0f;
	//	}
	//	else if (N.X == 1 && N.Y == 0) {
	//		N.g = 1.0f;
	//		N.rhs = 4.0f;
	//		//N.key.first = 1.0f;
	//		//N.key.second = 4.0f;
	//	}
	//	else if (N.X == 2 && N.Y == 0) {
	//		N.key.first = 2.0f;
	//		N.key.second = 0.0f;
	//	}
	//}
	print_queue(queue);

	std::cin.get();
}