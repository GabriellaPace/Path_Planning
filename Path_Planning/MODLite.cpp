#include "Functions.h"

/////////////////////////////////////////////////////////////////////////////////////
std::vector<Wptr_toNode> nonDomSuccs;
std::vector<Wptr_toNode> solutionPaths;

/*---------------------------------------------------------------------------------*/

	// expanding a state = observe the domination between g and rhs
	//Ns = node to expand, s1 = nondominated successor of Ns  (s1 = s’ ,  s2 = s’’)
std::vector<Wptr_toNode> generateMOPaths() {  //function GENERATE_MO_PATHS()  
	//// DECLARATIONS
	//Deq expandingStates;	// (de)queue of (ptr to) nodes which adjacents should be updated = to expand (FIFO)
	//std::vector<uint8_t> cumulativeCs;
	//uint8_t cost_tmp;

	//
	//// FIRST phase (from start to goal) 
	//expandingStates.push_back(ptrToStart);

	//	while ( !expandingStates.empty() ) {
	//		//re-initializations
	//		cumulativeCs.clear();	// right?? <---------------------------------------------------------------------------------------

	//		//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
	//		//Sptr_toNode Ns = expandingStates.front();
	//		Wptr_toNode Ns = expandingStates.front();
	//		expandingStates.pop_front();

	//		//nonDomSuccs = nonDom_[s' in succ(Ns)](sum(c(Ns, s’), g(s’))   <->   find non-dominated successors, wrt multiobjective c+g
	//		nonDomSuccs = nonDom_succs(Ns);
	//		
	//		for (auto s1 : nonDomSuccs) {
	//			if ((*Ns).parents.empty()) {							// if Ns doesn't have any parent (only iff s=Start): 		
	//				(*s1).parents[Ns] = compute_cost(Ns, s1);	// ^ for sure s' does not have any parent as well!
	//																	// ^ so Ns is added as a parent of s' with corresponding cost c(s, s').
	//			}
	//			else {													// if Ns does have predefined parents
	//				/*10*/
	//				cost_tmp = compute_cost(Ns, s1);
	//				for (auto& [s1_ptr, s1_cost] : (*Ns).parents) {
	//					cumulativeCs.push_back(cost_tmp + s1_cost);
	//				}

	//				/*11-12*/
	//				if ((*s1).parents.empty()) {
	//					(*s1).parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeC);
	//												// [!!!!] ASSUMING THAT HERE WE ALWAYS HAVE ONLY ONE VALUE IN cumulativeCs 

	//						std::cout << "cumulativeCs:\n"; // debug
	//						print_intVect(cumulativeCs); // debug
	//				}
	//				else {
	//			//		for (auto& [s2_ptr, s2_cost] : (*s1).parents) {		//for (auto s'' : s'.parents() ) {  
	//			//			if (s2_cost >= cumulativeC) {    
	//			//			//if ( equals(s1.parents(s2), cumulativeC) || completelyDominates(s1.parents(s2), cumulativeC) ) {
	//			//				break;
	//			//			}
	//			//			else if (s2_cost < cumulativeC) {
	//			//			//else if ( completelyDominates(cumulativeC, s1.parents(s2)) ) {
	//			//				(*s1).parents.erase(s2_ptr);		//s1.parents().remove(s2);
	//			//				(*s1).parents[Ns] = cumulativeC;	//s1.parents().put(s, cumulativeC);
	//			//			}
	//			//			else {
	//			//		//		for (auto cC : cumulativeC) {
	//			//		//			for (auto eC : (*Ns).parents(s)) {
	//			//		//				if (eC.equals(cC) || eC.dominates(cC)) {  //OR
	//			//		//					cumulativeC.remove(cC);
	//			//		//					break;
	//			//		//				}
	//			//		//				else if (cC.dominates(eC)) {
	//			//		//					s1.parents(s2).remove(eC);
	//			//		//					break;
	//			//		//				}
	//			//		//			}
	//			//		//			if (s1.parents(s2) == null) {
	//			//		//				s1.parents().remove(s2);
	//			//		//			}
	//			//		//		}
	//			//		//		if (cumulativeC != null) {
	//			//		//			s1.parents().put(s, cumulativeC);
	//			//		//		}
	//			//			}
	//			//		}
	//				}
	//			}

	//			//if (s1.parents.contains(s) && !expandingStates.contains(s1) )  =  Ns is among s' parents  and  s' is not already in the expanding queue
	//			if ( ((*s1).parents.find(Ns) != (*s1).parents.end())  &&  
	//				 (find(expandingStates.begin(), expandingStates.end(), s1) == expandingStates.end()) ) {
	//			
	//				expandingStates.push_back(s1);
	//			}
	//		}
	//	}

		// SECOND phase (from goal to start)
		//solutionPaths = construct paths recursively traversing parents;
		return solutionPaths;
}

/////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////   INITIALIZATIONS   ////////////////////////////////
//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;   // filled with Nodes, NOT ptr_to_Nodes !!
bool changed_costs = false;
Wptr_toNode N_inOld = nullptr;
std::vector<Wptr_toNode> ChangedNodes;

int main() {
	ReadMap_firstTime();
	for (auto N_ptr : NodesVect) {    // fill (and print) adjacents to each node 
		//(*N_ptr).findAdjacents();
		findAdjacents(N_ptr);
		(*N_ptr).print_Adjacents(); //debug
	} //(can't be done in constructor because not all nodes have been registered yet)

// function PLAN()
	// function Initialize()
	for (auto N_ptr : NodesVect) {
		if ( (*N_ptr).nodeType == goal ) {
			//(*N_ptr).calculateKey();
			calculateKey(N_ptr);
			queue.push(*N_ptr);
		}
	}
		print_queue(queue);  //debug

		//std::system("CLS");//debug
		printAll_g_rhs();    //debug
	queue = computeMOPaths(queue);
		printAll_g_rhs();	 //debug
		print_queue(queue);  //debug


	//while (start != goal) {
		solutionPaths = generateMOPaths();
		if (solutionPaths.empty()) {
			std::cout << " => There are no avaliable paths - waiting for any edge cost to change.\n\n";
		}
		
		ReadMap();	//second map  (//wait for any weight cost to change)
		for (auto d_ptr : newMap) {
			N_inOld = findNodeptr((*d_ptr).X, (*d_ptr).Y);
			if (N_inOld == nullptr) {	//Node not found
				std::cout << " The coordinates are not in the old map, so a new node will be created.\n\n";
				Node n9(d_ptr->Name, d_ptr->X, d_ptr->Y, d_ptr->cost, d_ptr->nodeType);  //define new Node <=======================
				changed_costs = true;
			}
			else {						//Node found
				if ((*d_ptr).cost != (*N_inOld).cost) {	//changed node cost!  //<==================================================
					changed_costs = true;
					ChangedNodes.push_back(N_inOld);	//save pointers of changed ones
				}
			}			
			
			if (changed_costs) {
				if ((*d_ptr).nodeType == start) {
					ptrToStart = findNodeptr((*d_ptr).X, (*d_ptr).Y);
				}
				if ((*d_ptr).nodeType == goal) {
					ptrToGoal = findNodeptr((*d_ptr).X, (*d_ptr).Y);
				}
			}
		}

		if (changed_costs) {
			//Node::k_m = Node::k_m + (*(ptrToGoal)).heuristic();  //start node has changed		
			k_m = k_m + heuristic(ptrToGoal);  //start node has changed
			for (auto cN_ptr : ChangedNodes) {	//= for all changed weight costs of NODES
				N_inOld = findNodeptr(cN_ptr->X, cN_ptr->Y);
				N_inOld->cost = cN_ptr->cost;	// = "Update cost" /*11*/
				//N_inOld->update_rhs();			// = "Update Vertex" /*12*/
				update_rhs(N_inOld);			// = "Update Vertex" /*12*/
				
				//N_inOld->updateAdjacents();		//should I update all the adjacent nodes' rhs??  <===============================
				updateAdjacents(N_inOld);		//should I update all the adjacent nodes' rhs??  <===============================

				queue = computeMOPaths(queue); // ok?  <=========================================================================
			}
		}
		changed_costs = false;
		ChangedNodes.clear();
	//}
// end of function PLAN()


		printAll_g_rhs();
		print_queue(queue);


	//// DELETE ALL objects
	//for (auto N_ptr : NodesVect) {
	//	delete &N_ptr;
	//	N_ptr = nullptr;
	//}
	std::cin.get();
}