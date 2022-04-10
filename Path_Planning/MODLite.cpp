#include "ReadMap.h"

// definition of Node's static variables
float Node::k_m = 0.0f;
std::shared_ptr<Node> Node::ptrToStart = nullptr;
/*--------------------------------------------------------------------------*/

using Qe = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
void print_queue(Qe q) {					// debug
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
///// functions to implement: /////
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
	for (auto N_ptr : Node::NodesVect) {    // fill (and print) adjacents to each node
		(*N_ptr).findAdjacents();
		(*N_ptr).print_Adjacents(); //debug
	}

	// function Initialize()
	for (auto N_ptr : Node::NodesVect) {
		if ( (*N_ptr).nodeType == goal ) {
			(*N_ptr).calculateKey();
			queue.push(*N_ptr);
		}
	}
	print_queue(queue); //debug


/*--------------------------------------------------------------------------------------------------------------------*/
	std::system("CLS");
	int idx;
	for (auto N_ptr : Node::NodesVect) {
		(*N_ptr).print_g_rhs();
	}
	std::cout << std::endl << " ---------------------- \n" << std::endl;

	auto itt = find_if(Node::NodesVect.begin(), Node::NodesVect.end(), 
			   [](const std::shared_ptr<Node>& objj) {return ((*objj).X == 0 && (*objj).Y == 1); });
	if (itt != Node::NodesVect.end()) {
		idx = std::distance(Node::NodesVect.begin(), itt);
		//(Node::AdjacentsList).push_back(Node::NodesVect[idx]);
		(*(Node::NodesVect[idx])).g = 30.0f;
	}

	(*(Node::NodesVect[idx])).updateAdjacents();
	for (auto N_ptr : Node::NodesVect) {
		(*N_ptr).print_g_rhs();
	}
/*--------------------------------------------------------------------------------------------------------------------*/


	//computeMOPaths(queue);

	std::cin.get();
}


// for weak pointers (check):
/*	if (auto temp = N_ptr.lock()) { // if Jack there
		std::cout << "ok";
	}
	else {
		std::cout << "The object is not there.";
	} */