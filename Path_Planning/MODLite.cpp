#include "Functions.h"

/////////////////////////////////////////////////////////////////////////////////////
std::vector<Wptr_toNode> nonDomSuccs;
std::vector<Wptr_toNode> solutionPaths;

/*---------------------------------------------------------------------------------*/

	// expanding a state = observe the domination between g and rhs
	// Ns = node to expand, s1 = nondominated successor of Ns  (s1 = s� ,  s2 = s��)
std::vector<Wptr_toNode> generateMOPaths() {  //function GENERATE_MO_PATHS()  
	// DECLARATIONS
	Deq expandingStates;	// (de)queue of (ptr to) nodes which adjacents should be updated = to expand (FIFO)
	std::vector<uint8_t> cumulativeCs;
	uint8_t cost_tmp;

	
	// FIRST phase (from start to goal) 
	expandingStates.push_back(ptrToStart);

		while ( !expandingStates.empty() ) {
			//re-initializations
			cumulativeCs.clear();	// right?? <=======================================================================================

			//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
			Wptr_toNode Ns = expandingStates.front();
			expandingStates.pop_front();

			//nonDomSuccs = nonDom_[s' in succ(Ns)](sum(c(Ns, s�), g(s�))   <->   find non-dominated successors, wrt multiobjective c+g
			nonDomSuccs = nonDom_succs(Ns);
			
			for (auto s1 : nonDomSuccs) {
				if (Ns->parents.empty()) {					// if Ns doesn't have any parent (only iff s=Start): 		
					s1->parents[Ns] = compute_cost(Ns, s1);	// ^ for sure s' does not have any parent as well!
					//(s1->parents[Ns]).push_back(compute_cost(Ns, s1));
																// ^ so Ns is added as a parent of s' with corresponding cost c(s, s').
				}
				else {											// if Ns does have predefined parents
					/*10*/
					cost_tmp = compute_cost(Ns, s1);
					for (auto& [s1_ptr, s1_cost] : (*Ns).parents) {
						cumulativeCs.push_back(cost_tmp + s1_cost);
					}

					/*11-12*/
					if ((*s1).parents.empty()) {
						(*s1).parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeC);
						std::cout << "cumulativeCs for parents of ";	Ns->print_Coord();	std::cout << std::endl; // debug
						print_intVect(cumulativeCs); // debug
					}
					else {
						for (auto& [s2_ptr, s2_cost] : (*s1).parents) {		//for (auto s'' : s'.parents() ) {  
							if (multi_dom(s2_cost, cumulativeCs) == areEqual || multi_dom(s2_cost, cumulativeCs) == fst_dominates){  
							//if ( equals(s1.parents(s2), cumulativeC) || completelyDominates(s1.parents(s2), cumulativeC) ) {
								break;
							}
							else if (multi_dom(s2_cost, cumulativeCs) == snd_dominates) {
							//else if ( completelyDominates(cumulativeC, s1.parents(s2)) ) {
								s1->parents.erase(s2_ptr);			//s1.parents().remove(s2);
								s1->parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeC);
							}
							else {
								for (auto cC : cumulativeCs) {
									//for (auto eC : s1->parents[s2_ptr]) {
										uint8_t eC = s1->parents[s2_ptr];
										// if (eC.equals(cC) || eC.dominates(cC)) {
										if (domination(cC, eC) == areEqual  || domination(cC, s1->parents[s2_ptr]) == snd_dominates ) {
											remove(cumulativeCs.begin(), cumulativeCs.end(), cC);	//cumulativeCs.erase(cC);
											break;
										}
										else if (domination(cC, eC) == fst_dominates) {
											s1->parents.erase(s2_ptr);	//s1.parents(s2_ptr).erase(eC);
											break;
										}
									//}					
									if (s1->parents.empty()) {	//if (s1.parents(s2_ptr) == null) {
										s1->parents.erase(s2_ptr);	//s1.parents().erase(s2_ptr);
									}
								}
								if (cumulativeCs.empty()) {	//if (cumulativeCs != null) {
									s1->parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeCs);
								}
							}
						}
					}
				}

				//if (s1.parents.contains(s) && !expandingStates.contains(s1) )
				if ( (s1->parents.find(Ns) != s1->parents.end())  &&
					 (find(expandingStates.begin(), expandingStates.end(), s1) == expandingStates.end()) ) {
					// =  Ns is among s' parents  and  s' is not already in the expanding queue
					expandingStates.push_back(s1);
				}
			}
		}

		// SECOND phase (from goal to start)
		//solutionPaths = construct paths recursively traversing parents;
		return solutionPaths;
}

/////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////   INITIALIZATIONS   ////////////////////////////////
//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;   // filled with Nodes, NOT ptr_to_Nodes !!
bool nodes_changes = false;
bool vehicle_moved = false;
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
				std::cout << " => Some coordinates were not in the old map, so a new node will be created in ";
				std::cout << "[" << d_ptr->X << "," << d_ptr->Y << "]\n\n";
				//Node n9(d_ptr->Name, d_ptr->X, d_ptr->Y, d_ptr->cost, d_ptr->nodeType);  //define new Node
				NodesVect.push_back(std::make_shared<Node>(d_ptr->Name, d_ptr->X, d_ptr->Y, d_ptr->cost, d_ptr->nodeType));	//define new Node
				nodes_changes = true;
			}
			else {						//Node found
				//the node changed its cost or type (start/goal/any):
				if (d_ptr->cost != N_inOld->cost  || d_ptr->nodeType != N_inOld->nodeType) {
					nodes_changes = true;
					ChangedNodes.push_back(N_inOld);	//save pointers of changed ones
				}
				//else: Node found but there were no modifications to it
			}			
			
			if (nodes_changes) {
				if (d_ptr->nodeType == start) {
					ptrToStart = findNodeptr(d_ptr->X, d_ptr->Y);
					vehicle_moved = true;
				}
				if (d_ptr->nodeType == goal) {
					ptrToGoal = findNodeptr(d_ptr->X, d_ptr->Y);
				}
			}
		}

		if (nodes_changes) {
			if (vehicle_moved) {	//start node has changed	
				k_m = k_m + heuristic(ptrToGoal);
				vehicle_moved = false;
			}
			
			for (auto cN_ptr : ChangedNodes) {	//= for all changed weight costs of NODES
				N_inOld = findNodeptr(cN_ptr->X, cN_ptr->Y);
				N_inOld->cost = cN_ptr->cost;	// = "Update cost" /*11*/
				N_inOld->nodeType = cN_ptr->nodeType;
				update_rhs(N_inOld);			// = "Update Vertex" /*12*/
				updateAdjacents(N_inOld);		//should I update all the adjacent nodes' rhs??  <===============================
			}
			queue = computeMOPaths(queue);	// /*13*/  ok? <=====================================================================
		}
		nodes_changes = false;
		ChangedNodes.clear();
	//}
// end of function PLAN()


		printAll_g_rhs();
		print_queue(queue);


	// DELETE ALL objects
		//shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	std::cin.get();
}