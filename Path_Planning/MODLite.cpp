#include "ReadMap.h"

using QT = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
//template<typename T>
void print_queue(QT q) {           // debug
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

	for (auto N : Node::NodesList) {
		N.calculateKey();
		N.heuristic();
		queue.push(N);
	}
	print_queue(queue);

	std::cin.get();
}