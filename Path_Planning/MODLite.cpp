#include "Functions.h"
//#define DEBUG  //#ifdef DEBUG   #endif


	// expanding a state = observe the domination between g and rhs
Qe expandingStates;  // queue of (ptr to) nodes which adjacents should be updated = to expand
std::vector<Sptr_toNode> nonDomSuccs;
// map parents() //keys = parents, values = cumulative costs
float cumulativeC;
std::vector<Sptr_toNode> solutionPaths;

	//Ns = node to expand, s1 = nondominated successor of Ns  (s1 = s’ ,  s2 = s’’)
std::vector<Sptr_toNode> generateMOPaths(){  //function GENERATE_MO_PATHS()   
	
	// FIRST phase (from start to goal)
	expandingStates.push(*(Node::ptrToStart));

		while ( !expandingStates.empty() ) {
			//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
			Sptr_toNode Ns = findNodeptr( expandingStates.top().X, expandingStates.top().Y);
			expandingStates.pop(); // ?


			//nonDomSuccs = nonDom_[s in succ(s)](sum(c(s, s’), g(s’))    <->    find non-dominated successors, wrt multiobjective c+g
			

			for (auto s1 : nonDomSuccs) {
				if ((*Ns).predecessor == nullptr) {		// if Ns doesn't have any parent (only iff s=StarT): 
														// ^ for sure s' does not have any parent as well!
					(*s1).predecessor = Ns;				// ^ so Ns is added as a parent of s' with corresponding cost c(s, s).
				}
				else {									// if Ns does have predefined parents
					//float c_v = std::accumulate(std::begin((*Ns).parents), std::end((*Ns).parents), 0, [](const std::float previous, const std::pair<const std::string, std::float>& p) { return previous + p.second; });
					//cumulativeC =cost(s, s1) + c_v;		// cumulative cost for s'

				//	if (s1.predecessor == nullptr) {
				//		s1.parents().put(s, cumulativeC);
				//	}
				//	else {
						//for (auto s : s.parents() ) {
						//	if ( equals(s1.parents(s2), cumulativeC) || completelyDominates(s1.parents(s2), cumulativeC) ) { //OR
						//		break;
						//	}
						//	else if ( completelyDominates(cumulativeC, s1.parents(s2)) ) {
						//		s1.parents().remove(s2);
						//		s1.parents().put(s, cumulativeC);
						//	}
						//	else {
						//		for (auto cC : cumulativeC) {
						//			for (auto eC : s.parents(s)) {
						//				if (eC.equals(cC) || eC.dominates(cC)) {  //OR
						//					cumulativeC.remove(cC);
						//					break;
						//				}
						//				else if (cC.dominates(eC)) {
						//					s1.parents(s2).remove(eC);
						//					break;
						//				}
						//			}
						//			if (s1.parents(s2) == null) {
						//				s1.parents().remove(s2);
						//			}
						//		}
						//		if (cumulativeC != null) {
						//			s1.parents().put(s, cumulativeC);
						//		}
						//	}
						//}
					//}
				}
				//if (s1.parents.contains(s) && !expandingStates.contains(s1) ) {
				//	expandingStates.push_back(s1);
				//}
			}
		}

		// SECOND phase (from goal to start)
		//solutionPaths = construct paths recursively traversing parents;
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