//#include "ClassNode.h"
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


// initialization of static variables for class Node:
//float Node::k_m = 0.0f;
//Node* Node::ptrToStart = nullptr;


int main() {
	//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;

	///////////////////////////////////////////////////////
	ReadMap();

	for (auto N : Node::NodesList) {
		queue.push(N);
		//std::cout << "pushed" << std::endl;
	}

	print_queue(queue);

	///////////////////////////////////////////////////////
	//n1.heuristic();
	//n2.heuristic();
	//n3.heuristic();
	///////////////////////////////////////////////////////
	std::cin.get();
}