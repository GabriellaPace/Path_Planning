#include "ReadMap.h"

// definition of Node's static variables
float Node::k_m = 0.0f;
int Node::X_start = -1;
int Node::Y_start = -1;
/*--------------------------------------------------------------------------*/

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
/*--------------------------------------------------------------------------*/

//void DomAll(Node a, Node b) {}

/*
void computeMOPaths(QT queue) {
	while (st > queue.top()) {
		Node deqN_withOldKey = queue.top();  //pick top one   // de-queued Node 
		Node deqN_withNewKey = queue.pop();  //pick and remove top one
		//Node::NodesList.remove(deqN_withNewKey);
		std::remove(Node::NodesList.begin(), Node::NodesList.end(), deqN_withNewKey);
		deqN_withNewKey.calculateKey();
		Node::NodesList.push_back(deqN_withNewKey);
		
		if (deqN_withOldKey > deqN_withNewKey) {
			queue.push(deqN_withNewKey);
		}
		else if (deqN_withNewKey.rhs > deqN_withNewKey.g) {		 // OVERCONSISTENT
			deqN_withNewKey.g = deqN_withNewKey.rhs;
			N.updateAdjacents;
		}
		else if (deqN_withNewKey.rhs < deqN_withNewKey.g) {		// UNDERCONSISTENT
			deqN_withNewKey.g = std::numeric_limits<float>::infinity();
			N.updateAdjacents;
			N.update;
		}
		else {									 // not dominant and not dominated 
			deqN_withNewKey.g = nonDom(deqN_withNewKey.g, deqN_withNewKey.rhs);
			N.updateAdjacents;
		}
	}
} */

/*--------------------------------------------------------------------------*/

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

	//computeMOPaths(queue);


	print_queue(queue); //debug

	std::cin.get();
}