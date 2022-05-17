#include "Functions.h"

/////////////////////////////////////////////////////////////////////////////////////


/*---------------------------------------------------------------------------------*/

	// expanding a state = observe the domination between g and rhs
	// Ns = node to expand, s1 = nondominated successor of Ns  (s1 = s’ ,  s2 = s’’)
std::vector<Wptr_toNode> generateMOPaths() {  //function GENERATE_MO_PATHS()  
	// DECLARATIONS
	Deq expandingStates;	// (de)queue of (ptr to) nodes which adjacents should be updated = to expand (FIFO)
	std::vector<Wptr_toNode> nonDomSuccs;
	std::vector<Wptr_toNode> solutionPaths;
	std::vector<uint8_t> cumulativeCs;
	uint8_t cost_tmp;

	
	// FIRST phase (from start to goal) 
	expandingStates.push_back(ptrToStart);

		while ( !expandingStates.empty() ) {
			//re-initializations
			cumulativeCs.clear();	// right?? <=======================================================================================

			//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
			Wptr_toNode Ns = expandingStates.front();
			//expandingStates.pop_front();

			//nonDomSuccs = nonDom_[s' in succ(Ns)](sum(c(Ns, s’), g(s’))   <->   find non-dominated successors, wrt multiobjective c+g
			nonDomSuccs = nonDom_succs(Ns);
			
			for (auto s1 : nonDomSuccs) {
				print_parents(Ns);
				if (Ns->parents.empty()) {					// if Ns doesn't have any parent (only iff s=Start): 		
					s1->parents[Ns] = compute_cost(Ns, s1);	// ^ for sure s' does not have any parent as well!
					//(s1->parents[Ns]).push_back(compute_cost(Ns, s1));
																// ^ so Ns is added as a parent of s' with corresponding cost c(s, s').
					print_parents(s1);
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
										if (domination(cC, eC) == areEqual  || domination(cC, eC) == snd_dominates ) {
												//print_intVect(cumulativeCs);
											std::remove(cumulativeCs.begin(), cumulativeCs.end(), cC);	//cumulativeCs.erase(cC);
												//print_intVect(cumulativeCs);
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
			expandingStates.pop_front();
		}

		// SECOND phase (from goal to start)
		//solutionPaths = construct paths recursively traversing parents;
		return solutionPaths;
}











std::vector<Wptr_toNode> solutionPaths;


int main() {
	ReadMap_firstTime();
	for (auto N_ptr : NodesVect) {    // fill (and print) adjacents to each node 
		findAdjacents(N_ptr);
		(*N_ptr).print_Adjacents(); //debug
	} //(can't be done in constructor because not all nodes have been registered yet)

// function PLAN()
	// function Initialize()
	for (auto N_ptr : NodesVect) {
		if ( (*N_ptr).nodeType == goal ) {
			calculateKey(N_ptr);
			//queue.push(*N_ptr);
			queue.insert(*N_ptr);
		}
	}

		printAll_g_rhs();   print_queue();  //debug
	computeMOPaths();
		
		//std::system("CLS");//debug

	//while (start != goal) {
		//solutionPaths = generateMOPaths();
		//if (solutionPaths.empty()) {
		//	std::cout << " => There are no avaliable paths - waiting for any edge cost to change.\n\n";
		//}
		////sleep(5);
		//updateMap();
	//}
// end of function PLAN()

		//printAll_g_rhs();   print_queue();  //debug

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	std::cin.get();
}