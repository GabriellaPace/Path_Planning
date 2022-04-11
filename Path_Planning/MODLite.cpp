#include "ReadMap.h"

using Sptr_toNode = std::shared_ptr<Node>;

// definition of Node's static variables
float Node::k_m = 0.0f;
Sptr_toNode Node::ptrToStart = nullptr;
/*--------------------------------------------------------------------------*/

using Qe = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
void print_queue(Qe q) {					// debug
	std::cout << "Queue:" << std::endl;
	Node tmp;
	if (q.empty())
		std::cout << " empty!";
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
	for (auto N_ptr : Node::NodesVect) {
		(*N_ptr).print_g_rhs();
	}
	std::cout << std::endl;
}

/*--------------------------------------------------------------------------*/

Sptr_toNode findNodeidx(int xx, int yy) {    // find the index of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx = -1;  //this should raise an error if used in a vector
	auto it = find_if(Node::NodesVect.begin(), Node::NodesVect.end(), 
			   [&x, &y](const std::shared_ptr<Node>& obj) {return ((*obj).X == x && (*obj).Y == y); });
	if (it != Node::NodesVect.end()) {  //shouln't be needed, but just in case
		idx = (int) std::distance(Node::NodesVect.begin(), it);
	}
	return Node::NodesVect[idx];
}


Qe computeMOPaths(Qe queue) {
	(*(Node::ptrToStart)).calculateKey();
	while ( !queue.empty()  &&  *(Node::ptrToStart) < queue.top() ) {	// = start.key dominated the top key in the queue
		Node deqN_wOldKey = queue.top();  //pick top one (deqN = de-queued Node)
		queue.pop();					  //and then remove it
		
		Sptr_toNode deqN_ptr = findNodeidx(deqN_wOldKey.X, deqN_wOldKey.Y);	 //ptr to de-queued node	
		(*deqN_ptr).calculateKey();
		
		if (deqN_wOldKey > *deqN_ptr) {					 //put it back in queue with new key
			queue.push(*deqN_ptr);
		}
		else if ((*deqN_ptr).rhs > (*deqN_ptr).g) {		 // OVERCONSISTENT
			(*deqN_ptr).g = (*deqN_ptr).rhs;
			(*deqN_ptr).updateAdjacents();
		}
		else if ((*deqN_ptr).rhs < (*deqN_ptr).g) {		 // UNDERCONSISTENT
			(*deqN_ptr).g = std::numeric_limits<float>::infinity();
			(*deqN_ptr).updateAdjacents();
			(*deqN_ptr).update_rhs();
		}
		else {											 // not dominant and not dominated 
			(*deqN_ptr).g = nonDom((*deqN_ptr).g, (*deqN_ptr).rhs);
			(*deqN_ptr).updateAdjacents();
		}

		(*(Node::ptrToStart)).calculateKey(); //for next loop
	}
	return queue;
}

//void DomAll(Node a, Node b) {}

/*--------------------------------------------------------------------------*/


int main() {
	//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;   // filled with Nodes, NOT ptr_to_Nodes !!

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
	////std::system("CLS");
	//int idx;
	//for (auto N_ptr : Node::NodesVect) {
	//	(*N_ptr).print_g_rhs();
	//}
	//std::cout << std::endl << " ---------------------- \n" << std::endl;
	//auto itt = find_if(Node::NodesVect.begin(), Node::NodesVect.end(), 
	//		   [](const std::shared_ptr<Node>& objj) {return ((*objj).X == 0 && (*objj).Y == 1); });
	//if (itt != Node::NodesVect.end()) {
	//	idx = std::distance(Node::NodesVect.begin(), itt);
	//	(*(Node::NodesVect[idx])).g = 30.0f;
	//}
	//(*(Node::NodesVect[idx])).updateAdjacents();
	//for (auto N_ptr : Node::NodesVect) {
	//	(*N_ptr).print_g_rhs();
	//}
/*--------------------------------------------------------------------------------------------------------------------*/

	//std::system("CLS");
	printAll_g_rhs();
	queue = computeMOPaths(queue);
	std::cout << " [computing MO Path: done]\n\n";

	printAll_g_rhs();
	print_queue(queue); //debug

	std::cin.get();
}


// for weak pointers (check):
/*	if (auto temp = N_ptr.lock()) { // if Jack there
		std::cout << "ok";
	}
	else {
		std::cout << "The object is not there.";
	} */