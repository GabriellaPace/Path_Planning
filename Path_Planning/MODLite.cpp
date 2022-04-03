//#include "ClassNode.h"
#include "ReadMap.h"


template<typename T>
void print_queue(T& q) {           // debug
	Node tmp;
	while (!q.empty()) {
		tmp = q.top();
		tmp.print_NodeKey();
		q.pop();
	}
	std::cout << std::endl;
}


// initialization of static variables for class Node:
float Node::k_m = 0.0f;
Node* Node::ptrToStart = nullptr;
std::list<Node*> Node::allNodes;



int main() {
	//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node> >  queue;

	///////////////////////////////////////////////////////
	ReadMap();

	std::list<Node*>::const_iterator it;
	for (it = (Node::allNodes).begin(); it != (Node::allNodes).end(); ++it) {
		queue.push(**it);
		//std::cout << "pushed" << std::endl;
	};

	print_queue(queue);

	///////////////////////////////////////////////////////
	//n1.heuristic();
	//n2.heuristic();
	//n3.heuristic();

	///////////////////////////////////////////////////////

	std::cin.get();
}