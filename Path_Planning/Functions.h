#pragma once
#include "ReadMap.h"

using Sptr_toNode = std::shared_ptr<Node>;

// definition of Node's static variables
float Node::k_m = 0.0f;
Sptr_toNode  Node::ptrToStart = nullptr;
Sptr_toNode  Node::ptrToGoal = nullptr;


using Qe = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
using Deq = std::deque<Sptr_toNode>;

/*-------------------------------  Debug functions  -------------------------------*/
void print_queue(Qe q) {					// debug
	std::cout << "Queue:" << std::endl;
	Node tmp;
	if (q.empty())
		std::cout << " empty!\n";
	else {
		while (!q.empty()) {
			tmp = q.top();
			tmp.print_NodeKey();
			q.pop();
		}
	}
	std::cout << std::endl;
}

void printAll_g_rhs() {
	std::cout << "g and rhs for all current nodes:\n";
	for (auto N_ptr : Node::NodesVect) {
		(*N_ptr).print_g_rhs();
	}
	std::cout << std::endl;
}

void print_intVect(std::vector<uint8_t> vect) {
	for (auto& v : vect) {
		std::cout << v << std::endl;
	}
	std::cout << std::endl;
}

/*----------------------------------  Functions  ----------------------------------*/
Sptr_toNode findNodeptr(int xx, int yy) {    // find the pointer of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx; // = -1;   //<- this should raise an error if used in a vector
	auto it = find_if(Node::NodesVect.begin(), Node::NodesVect.end(),
		[&x, &y](const std::shared_ptr<Node>& obj) {return ((*obj).X == x && (*obj).Y == y); });
	if (it != Node::NodesVect.end()) {
		idx = (int)std::distance(Node::NodesVect.begin(), it);
		return Node::NodesVect[idx];
	}
	else {
		return nullptr;
	}
}



uint8_t compute_cost(Sptr_toNode n1, Sptr_toNode n2) {	// edge-cost derived from node-costs
	return std::max(n1->cost, n2->cost);	//<- as done for Theta* Planner in Nav2
}



std::vector<Sptr_toNode> nonDom_succs(Sptr_toNode N) {		// find non-dominated nodes among successors of the given node		
	std::vector<Sptr_toNode> nonDomSuccs_tmp;
	bool nonDom_flag;
	float cC_out, cC_in;  // still considering single g and rhs -> will become vectors //cC = cumulative cost (outer/inner loop)

	for(auto adN : N->AdjacentsList) {		//per ogni elemento di AdjacentsList
		nonDom_flag = true;
		cC_out = compute_cost(N, adN) + adN->g;
		
		for (auto inN : N->AdjacentsList) {		//paragona con goni altro elemento di AdjacentsList [anche con se stesso!! -> problema?	 
			cC_in = compute_cost(N, inN) + inN->g;
			if (!nonDom_b(cC_out, cC_in)) {			//it is dominated by someone-else!
				nonDom_flag = false;
				break;
			}
		}

		if (nonDom_flag) {
			nonDomSuccs_tmp.push_back(adN);
		}
	}

	return nonDomSuccs_tmp;
}



Qe computeMOPaths(Qe queue) {  //function COMPUTE_MO_PATHS()	
	(*(Node::ptrToStart)).calculateKey();
	while (!queue.empty() && *(Node::ptrToStart) < queue.top()) {	// = start.key dominated the top key in the queue
		Node deqN_wOldKey = queue.top();  //pick top one (deqN = de-queued Node)
		queue.pop();					  //and then remove it

		Sptr_toNode deqN_ptr = findNodeptr(deqN_wOldKey.X, deqN_wOldKey.Y);	 //ptr to de-queued node	
		(*deqN_ptr).calculateKey();

		if (deqN_wOldKey < *deqN_ptr) {					 //put it back in queue with new key
			queue.push(*deqN_ptr);
		}
		else if ((*deqN_ptr).rhs < (*deqN_ptr).g) {		 // OVERCONSISTENT
			(*deqN_ptr).g = (*deqN_ptr).rhs;
			(*deqN_ptr).updateAdjacents();
		}
		else if ((*deqN_ptr).rhs > (*deqN_ptr).g) {		 // UNDERCONSISTENT
			(*deqN_ptr).g = std::numeric_limits<float>::infinity();
			(*deqN_ptr).updateAdjacents();
			(*deqN_ptr).update_rhs();
		}
		else {											 // not dominant and not dominated 
			(*deqN_ptr).g = nonDom_2((*deqN_ptr).g, (*deqN_ptr).rhs);
			(*deqN_ptr).updateAdjacents();
		}

		(*(Node::ptrToStart)).calculateKey(); //for next loop
	}
	std::cout << " => Computed MO Paths.\n\n";
	return queue;
}