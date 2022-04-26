#include "ReadMap.h"
//#define DEBUG  //#ifdef DEBUG   #endif

using Sptr_toNode = std::shared_ptr<Node>;
// definition of Node's static variables
float Node::k_m = 0.0f;
Sptr_toNode  Node::ptrToStart = nullptr;
Sptr_toNode  Node::ptrToGoal  = nullptr;


//////////////////////////////////    FUNCTIONS   ///////////////////////////////////
/*-------------------------------  Debug functions  -------------------------------*/
using Qe = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
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

/*----------------------------------  Functions  ----------------------------------*/
Sptr_toNode findNodeptr(int xx, int yy) {    // find the pointer of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx; // = -1;   //<- this should raise an error if used in a vector
	auto it = find_if(Node::NodesVect.begin(), Node::NodesVect.end(), 
			   [&x, &y](const std::shared_ptr<Node>& obj) {return ((*obj).X == x && (*obj).Y == y); });
	if (it != Node::NodesVect.end()) {
		idx = (int) std::distance(Node::NodesVect.begin(), it);
		return Node::NodesVect[idx];
	}
	else {
		return nullptr;
	}
}


Qe computeMOPaths(Qe queue) {  //function COMPUTE_MO_PATHS()	
	(*(Node::ptrToStart)).calculateKey();
	while ( !queue.empty()  &&  *(Node::ptrToStart) < queue.top() ) {	// = start.key dominated the top key in the queue
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
			(*deqN_ptr).g = nonDom((*deqN_ptr).g, (*deqN_ptr).rhs);
			(*deqN_ptr).updateAdjacents();
		}

		(*(Node::ptrToStart)).calculateKey(); //for next loop
	}
	std::cout << " => Computed MO Paths.\n\n";
	return queue;
}


//// expanding a state = observe the domination between g and rhs
//Qe expandingStates;  // queue of (ptr to) nodes which adjacents should be updated = to expand
//std::vector<Sptr_toNode> nonDomSuccs;
//// map parents() //keys = parents, values = cumulative costs
std::vector<Sptr_toNode> solutionPaths;
////? cumulativeC;
//
//
////s = node to expand, s1 = nondominated successor of s
//
std::vector<Sptr_toNode> generateMOPaths(){  //function GENERATE_MO_PATHS()   -   s1 = s’ ,  s2 = s’’
//	
//	// FIRST phase (from start to goal)
//	expandingStates.push(Node::ptrToStart);
//
//		while ( !expandingStates.empty() ) {
//			//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
//			Sptr_toNode Ns = expandingStates.top();
//			expandingStates.pop(); // ?
//
//
//			//nonDomSuccs = nonDom_[s in succ(s)](sum(c(s, s’), g(s’))    <->    find non-dominated successors, wrt multiobjective c+g
//			
//
//			for (auto s1 : nonDomSuccs) {
//				if (Ns.predecessor == nullptr) {      // if s doesn't have any parent (only iff s=StarT): 
//													  // ^ for sure s' does not have any parent as well!
//					s1.predecessor = *Ns;			  // ^ so Ns is added as a parent of s' with corresponding cost c(s, s).
//				}
//				else {
//				//	cumulativeC = sum(c(s, s1), s.parents().values());
//				//	if (s1.predecessor == nullptr) {
//				//		s1.parents().put(s, cumulativeC);
//				//	}
//				//	else {
//						//for (auto s : s.parents() ) {
//						//	if ( equals(s1.parents(s2), cumulativeC) || completelyDominates(s1.parents(s2), cumulativeC) ) { //OR
//						//		break;
//						//	}
//						//	else if ( completelyDominates(cumulativeC, s1.parents(s2)) ) {
//						//		s1.parents().remove(s2);
//						//		s1.parents().put(s, cumulativeC);
//						//	}
//						//	else {
//						//		for (auto cC : cumulativeC) {
//						//			for (auto eC : s.parents(s)) {
//						//				if (eC.equals(cC) || eC.dominates(cC)) {  //OR
//						//					cumulativeC.remove(cC);
//						//					break;
//						//				}
//						//				else if (cC.dominates(eC)) {
//						//					s1.parents(s2).remove(eC);
//						//					break;
//						//				}
//						//			}
//						//			if (s1.parents(s2) == null) {
//						//				s1.parents().remove(s2);
//						//			}
//						//		}
//						//		if (cumulativeC != null) {
//						//			s1.parents().put(s, cumulativeC);
//						//		}
//						//	}
//						//}
//					//}
//				}
//				//if (s1.parents.contains(s) && !expandingStates.contains(s1) ) {
//				//	expandingStates.push_back(s1);
//				//}
//			}
//		}
//
//		// SECOND phase (from goal to start)
//		//solutionPaths = construct paths recursively traversing parents;
		return solutionPaths;
}



////////////////////////////////   INITIALIZATIONS   ////////////////////////////////
//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;   // filled with Nodes, NOT ptr_to_Nodes !!
bool changed_edges = false;
//std::vector<Sptr_toNode> solutionPaths;
Sptr_toNode N_inOld = nullptr;
std::vector<Sptr_toNode> ChangedNodes;

/////////////////////////////////////////////////////////////////////////////////////

int main() {
	ReadMap_firstTime();
	for (auto N_ptr : Node::NodesVect) {    // fill (and print) adjacents to each node
		(*N_ptr).findAdjacents();
		(*N_ptr).print_Adjacents(); //debug
	}

// function PLAN()
	// function Initialize()
	for (auto N_ptr : Node::NodesVect) {
		if ( (*N_ptr).nodeType == goal ) {
			(*N_ptr).calculateKey();
			queue.push(*N_ptr);
		}
	}
		print_queue(queue);  //debug

		//std::system("CLS");//debug
		printAll_g_rhs();    //debug
	queue = computeMOPaths(queue);
		printAll_g_rhs();	 //debug
		print_queue(queue);  //debug


	//while (true) {
		solutionPaths = generateMOPaths();
		if (solutionPaths.empty()) {
			std::cout << " => There are no avaliable paths - waiting for any edge cost to change.\n\n";
		}
		
		ReadMap();	//second map  (//wait for any weight cost to change)
		for (auto d_ptr : dummyNode::newMap) {
			N_inOld = findNodeptr((*d_ptr).X, (*d_ptr).Y);
			if (N_inOld == nullptr) {  //Node not found
				std::cout << " The coordinates are not in the old map, so a new node will be created.\n\n";
				Node n9((*d_ptr).Name, d_ptr->X, d_ptr->Y, (*d_ptr).cost, d_ptr->nodeType);  //define new Node
				changed_edges = true;
			}
			else {					  //Node found
				if ((*d_ptr).cost != (*N_inOld).cost) {   //changed edge cost!
					changed_edges = true;
					ChangedNodes.push_back(N_inOld);  //save pointers of changed ones
				}
			}			
			
			if (changed_edges) {
				if ((*d_ptr).nodeType == start) {
					Node::ptrToStart = findNodeptr((*d_ptr).X, (*d_ptr).Y);
				}
				if ((*d_ptr).nodeType == goal) {
					Node::ptrToGoal = findNodeptr((*d_ptr).X, (*d_ptr).Y);
				}
			}
		}

		if (changed_edges) {
			Node::k_m = Node::k_m + (*(Node::ptrToGoal)).heuristic();  //start node has changed		
			for (auto cN_ptr : ChangedNodes) {  //= for all Changed weight costs of edges(u, v) {
				N_inOld = findNodeptr(cN_ptr->X, cN_ptr->Y);
				N_inOld->cost = cN_ptr->cost;  //= Update cost c(u, v);
				N_inOld->update_rhs();
				queue = computeMOPaths(queue); //<===============================================================================
			}
		}
		changed_edges = false;
		ChangedNodes.clear();
	//}
// end of function PLAN()


		printAll_g_rhs();
		print_queue(queue);


	//// DELETE ALL objects
	//for (auto N_ptr : Node::NodesVect) {
	//	delete &N_ptr;
	//	N_ptr = nullptr;
	//}
	std::cin.get();
}