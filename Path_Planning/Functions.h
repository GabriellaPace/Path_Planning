#pragma once
#include "ReadMap.h"


using set = std::set<Node, std::less<Node>, std::pmr::polymorphic_allocator<Node> >;	//by definition doesn't allow duplicates
	//std::pmr::polymorphic_allocator = allows objects to behave as if they used different allocator types despite the identical static allocator type
using Deq = std::deque<Wptr_toNode>;	//for expanding_states (unordered queue)
using parents_map = robin_hood::unordered_map < Wptr_toNode, uint8_t >;


set queue;	//filled with Nodes, NOT ptr_to_Nodes !!


////////////////////////////////////// Debug functions //////////////////////////////////////
void print_queue() {
	std::cout << "Queue:" << std::endl;
	if (queue.empty())
		std::cout << " empty!\n";
	else {
		for (auto N : queue)
		{
			N.print_NodeKey();
		}
	}
	std::cout << std::endl;
}

void printAll_g_rhs() {
	std::cout << "\ng and rhs for all current nodes:\n";
	for (auto N_ptr : NodesVect) {
		(*N_ptr).print_g_rhs();
	}
	std::cout << std::endl;
}

void print_intVect(std::vector<uint8_t> vect) {
	for (int i = 0; i < vect.size(); ++i) {
		std::cout << +vect[i] << std::endl;		//+ needed to print unsigned_int8
	}
	std::cout << std::endl;
}

void print_parents(Wptr_toNode N) {
	std::cout << "Parents of node ";	N->print_Coord();		std::cout << " :\n";
	for (parents_map::iterator it = N->parents.begin(); it != N->parents.end(); ++it) {
		std::cout << "Key (coord): ";	(it->first)->print_Coord();
		std::cout << "  -  Value: " << +(it->second) << std::endl;		//+ needed to print unsigned_int8
	}
	std::cout << std::endl;
}

void print_solution(std::vector<Wptr_toNode> solution_vect) {
	std::cout << "SOLUTION PATH for map {" << count-1 << "} optimized for g[1]:\n";
	for (auto ptr : solution_vect){
		ptr->print_Coord();
		std::cout << std::endl;
	}
	std::cout << std::endl;

	/**/
	for (int i = 0; i < 3; ++i)		NodesVect[i]->print_g_rhs();
	std::cout << std::endl;
	for (int i = 3; i < 6; ++i)		NodesVect[i]->print_g_rhs();
	std::cout << std::endl;
	/**/
}
///////////////////////////////////////// Functions /////////////////////////////////////////

Wptr_toNode findNodeptr(int xx, int yy) {    // find the pointer of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx;
	auto it = find_if(NodesVect.begin(), NodesVect.end(),
			  [&x, &y](const Wptr_toNode& obj) {return ((*obj).X == x && (*obj).Y == y); });
	if (it != NodesVect.end()) {
		idx = (int)std::distance(NodesVect.begin(), it);
		return NodesVect[idx];
	}
	else {
		return nullptr;
	}
}

/*----------------------------------------------------------------------------------------*/

int heuristic(Wptr_toNode N) {		// shortest aereal path (ignoring the grid)
	int X_start = (*ptrToStart).X;
	int Y_start = (*ptrToStart).Y;

	int h = (int)((sqrt(pow((N->X - X_start), 2.0f) + pow((N->Y - Y_start), 2.0f))) * 10);  //pow(base, power)
	//std::cout << "Heuristic of node [" << X << "," << Y << "] : " << h ;
	//std::cout << "   (-> wrt start node with coord: [" << X_start << "," << Y_start << "] )" << std::endl << std::endl;
	return h;
}


void calculateKey(Wptr_toNode N) {
	N->key.second = nonDom(N->g, N->rhs);
	N->key.first = N->key.second + heuristic(N) + k_m;
}


uint8_t compute_cost(Wptr_toNode n1, Wptr_toNode n2) {	// edge-cost derived from node-costs
	return std::max(n1->cost, n2->cost);	//<- as done for Theta* Planner in Nav2
}

/*------------------------------------- Nodes updates -------------------------------------*/

void addAdj(Wptr_toNode N, int oriz, int vert) {
	auto it = find_if(NodesVect.begin(), NodesVect.end(), [&oriz, &vert](const Wptr_toNode& obj) {return ((*obj).X == oriz && (*obj).Y == vert); });
	if (it != NodesVect.end()) {
		auto idx = std::distance(NodesVect.begin(), it);
		N->AdjacentsVect.push_back(NodesVect[idx]);
	}
}


void findAdjacents(Wptr_toNode N) {
	int oriz, vert;
	oriz = N->X + 0;	vert = N->Y + 1;	addAdj(N, oriz, vert);
						vert = N->Y - 1;	addAdj(N, oriz, vert);
	oriz = N->X + 1;	vert = N->Y + 0;	addAdj(N, oriz, vert);
	oriz = N->X - 1;						addAdj(N, oriz, vert);
	oriz = N->X + 1;	vert = N->Y + 1;	addAdj(N, oriz, vert);
	oriz = N->X - 1;						addAdj(N, oriz, vert);
	oriz = N->X + 1;	vert = N->Y - 1;	addAdj(N, oriz, vert);
	oriz = N->X - 1;						addAdj(N, oriz, vert);
}


struct NDS_struct {
	std::vector<Wptr_toNode> NDS_succs;
	float NDS_rhs;
};

struct NDS_struct nonDom_succs(Wptr_toNode N) {				// find non-dominated successors, wrt multiobjective c+g
															//nonDomSuccs = nonDom_[s' in succ(Ns)](sum(c(Ns, s’), g(s’)) 
	NDS_struct NDS_sol;
	bool nonDom_flag;
	Wptr_toNode adNi, adNj;
	float cC_out, cC_in;  // still considering single g and rhs -> will become vectors //cC = cumulative cost (outer/inner loop)

	for (auto adNi : N->AdjacentsVect) {		//for each element of AdjacentsVect, we'll check if it dominated every other element in the same List
		nonDom_flag = true;
		cC_out = compute_cost(N, adNi) + adNi->g;	

		for (auto adNj : N->AdjacentsVect) {		//paragona con ogni altro elemento di AdjacentsVect [anche con se stesso!!] -> problema? <============
			cC_in = compute_cost(N, adNj) + adNj->g;
			if (domination(cC_out, cC_in) == snd_dominates) {		//it is dominated by someone-else!	//if (!nonDom_b(cC_out, cC_in))
				nonDom_flag = false;
				break;
			}
		}
		if (nonDom_flag  &&  find(N->AdjacentsVect.begin(), N->AdjacentsVect.end(), adNi) != N->AdjacentsVect.end()) {	// to avoid duplicates
			NDS_sol.NDS_succs.push_back(adNi);
			NDS_sol.NDS_rhs = cC_out;	//c(N,s1) + g(s1)
		}
	}

	return NDS_sol;
}


void update_rhs(Wptr_toNode N) {    //function UPDATE_VERTEX(u)
	if (N->nodeType != goal) {
		N->rhs = nonDom_succs(N).NDS_rhs;
	}

	if (queue.find(*N) != queue.end()) {	// if N is in the queue, remove it
		queue.erase(*N);
	}

	if (domination(N->g, N->rhs) != areEqual) {
		calculateKey(N);
		queue.insert(*N);
	}
}


void updateAdjacents(Wptr_toNode N) {
	for (auto A_ptr : N->AdjacentsVect) {   //update each node adjacent to the modified one
		update_rhs(A_ptr);
	}
}


/*--------------------------------------- ROUTINES ----------------------------------------*/

void computeMOPaths() {  //function COMPUTE_MO_PATHS()	
	Node deqN_wOldKey;
	Wptr_toNode deqN_ptr;
	float inf = std::numeric_limits<float>::infinity();	// can't do "using"

	calculateKey(ptrToStart);

	//while (!queue.empty()  &&  ( (*ptrToStart).key.second == inf || *ptrToStart < *(queue.begin()) ) ) {	// termination criteria = start.key dominates the top key in the queue ( < = ordering rule, based on key)
	while (!queue.empty()  &&  !( *ptrToStart < *(queue.begin()) )) {	// termination criteria = start.key dominates the top key in the queue ( < : ordering rule, based on key)
		auto queue_top = queue.begin();
		deqN_wOldKey = *queue_top;  //pick top Node in the queue (deqN = de-queued Node)
		queue.erase(queue_top);		//equivalent to "queue.pop()" -> remove dequeued node from the queue

		deqN_ptr = findNodeptr(deqN_wOldKey.X, deqN_wOldKey.Y);	 //ptr to de-queued node
		calculateKey(deqN_ptr);

		if (deqN_wOldKey < *deqN_ptr) {					 //put it back in queue with new key  ->  when does this happen???
			queue.insert(*deqN_ptr);
		}
		//else if ((*deqN_ptr).rhs < (*deqN_ptr).g) {
		else if ( domination(deqN_ptr->rhs, deqN_ptr->g) == fst_dominates ) {		 // OVERCONSISTENT
			(*deqN_ptr).g = (*deqN_ptr).rhs;
			updateAdjacents(deqN_ptr);
		}
		//else if ((*deqN_ptr).rhs > (*deqN_ptr).g) {
		else if ( domination(deqN_ptr->rhs, deqN_ptr->g) == snd_dominates ) {		// UNDERCONSISTENT
			(*deqN_ptr).g = std::numeric_limits<float>::infinity();
			update_rhs(deqN_ptr);
			updateAdjacents(deqN_ptr);
		}
		else {											 // not dominant and not dominated 
			(*deqN_ptr).g = nonDom((*deqN_ptr).g, (*deqN_ptr).rhs);
			updateAdjacents(deqN_ptr);
		}

		calculateKey(ptrToStart); //for next loop (needed??)
	}
	std::cout << " => Computed MO Paths.\n\n";
}


std::vector<Wptr_toNode> generateMOPaths() {  //function GENERATE_MO_PATHS()  
//expanding a state = observe the domination between g and rhs
//Ns = node to expand, s1 = nondominated successor of Ns  (s1 = s’ ,  s2 = s’’)
	// DECLARATIONS
	Deq expandingStates;	// (de)queue of (ptr to) nodes which adjacents should be updated = to expand (FIFO)
	std::vector<Wptr_toNode> nonDomSuccs;
	std::vector<Wptr_toNode> solutionPaths;
	//std::vector<uint8_t> cumulativeCs;
	uint8_t cumulativeCs;

	//uint8_t cost_tmp;

/*-- FIRST phase (from start to goal) -----------------------------------------------------------*/
	expandingStates.push_back(ptrToStart);

	while (!expandingStates.empty()) {
		//re-initializations
		//cumulativeCs.clear();
		cumulativeCs = NULL;

		//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
		Wptr_toNode Ns = expandingStates.front();
		nonDomSuccs = nonDom_succs(Ns).NDS_succs;		// find non-dominated successors, wrt multiobjective c+g

		for (auto s1 : nonDomSuccs) {
			if (Ns->parents.empty()) {					// if Ns doesn't have any parent (only iff s=Start): 
														// ^ for sure s' does not have any parent as well!
				s1->parents[Ns] = compute_cost(Ns, s1);	// ^ so Ns is added as a parent of s' with corresponding cost c(s, s').		
			}
			else {										// if Ns does have predefined parents
				/*10*/
				//cost_tmp = compute_cost(Ns, s1);
				for (auto&[s1_ptr, s1_cost] : Ns->parents) {
					//cumulativeCs.push_back(cost_tmp + s1_cost);
					cumulativeCs += s1_cost;	//"aggregated" cost??
				}
				cumulativeCs += compute_cost(Ns, s1);

				/*11-12*/
				if (s1->parents.empty()) {
					//s1->parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeC);
					s1->parents[Ns] = cumulativeCs;
					//std::cout << "cumulativeCs for parents of ";	Ns->print_Coord();	std::cout << std::endl; // debug
					//print_intVect(cumulativeCs); // debug
				}
				else {
					for (auto&[s2_ptr, s2_cost] : s1->parents) {		//for (auto s'' : s'.parents() ) {  
						//if (multi_dom(s2_cost, cumulativeCs) == areEqual || multi_dom(s2_cost, cumulativeCs) == fst_dominates) {
						if (domination(s2_cost, cumulativeCs) == areEqual || domination(s2_cost, cumulativeCs) == fst_dominates) {

							break;
						}
						//else if (multi_dom(s2_cost, cumulativeCs) == snd_dominates) {
						else if (domination(s2_cost, cumulativeCs) == snd_dominates) {
							s1->parents.erase(s2_ptr);
							//s1->parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeC);
							s1->parents[Ns] = cumulativeCs;
						}
						else {
							//for (auto cC : cumulativeCs) {	// = for each type of cost
								//for (auto eC : s1->parents[s2_ptr]) {	// = for each element of the cost vector (=type of cost) (??)
								uint8_t eC = s1->parents[s2_ptr];
								uint8_t cC = cumulativeCs;
								if (domination(cC, eC) == areEqual || domination(cC, eC) == snd_dominates) {
									//std::remove(cumulativeCs.begin(), cumulativeCs.end(), cC);
									cumulativeCs = NULL;	//??????
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
							//}
							//if (! cumulativeCs.empty()) {	//if (cumulativeCs != null) {
							if (cumulativeCs != NULL) {	//if (cumulativeCs != null) {
								//s1->parents[Ns] = cumulativeCs[0];	//s1.parents().put(s, cumulativeCs);
								s1->parents[Ns] = cumulativeCs;
							}
						}
					}
				}
			}

			if ((s1->parents.find(Ns) != s1->parents.end()) &&
				(find(expandingStates.begin(), expandingStates.end(), s1) == expandingStates.end())) {
				// =  Ns is among s' parents  and  s' is not already in the expanding queue
				expandingStates.push_back(s1);
			}
		}
		expandingStates.pop_front();
	}


/*-- SECOND phase (from goal to start) ----------------------------------------------------------*/
		//solutionPaths = construct paths recursively traversing parents;
	Wptr_toNode N = ptrToGoal;
	solutionPaths.push_back(N);

	while (N->nodeType != start) {
		if (N->parents.empty()) {	//failed to gnerate a complete path
			solutionPaths.clear();
			return solutionPaths;
		}

		// (*) : repeat same for the other entries of the cost vector -> to obtain a path which each optimize one of the costs
		float min_g1 = std::numeric_limits<float>::infinity();
		Wptr_toNode par_toPush;
		for (auto&[par_ptr, par_cost] : N->parents) {
			if (par_cost < min_g1) {
				min_g1 = par_cost;
				par_toPush = par_ptr;
			}
		}
		solutionPaths.push_back(par_toPush);
		N = par_toPush;	//for next iteration
	}
	std::reverse(solutionPaths.begin(), solutionPaths.end());	//to have it from start to goal


	std::cout << " => Generated MO Paths.\n";
	return solutionPaths;
}


void updateMap() {
	// initializations
	bool nodes_changes = false, vehicle_moved = false;
	Wptr_toNode N_inOld = nullptr;

	ReadMap();	// to add -> wait for any weight cost to change

	for (auto d_ptr : newMap) {
		N_inOld = findNodeptr((*d_ptr).X, (*d_ptr).Y);
		if (N_inOld == nullptr) {	//Node not found
			std::cout << " => coordinates [" << d_ptr->X << "," << d_ptr->Y << "] were not in the old map, so a new node will be created.\n"; //debug		
			NodesVect.push_back(std::make_shared<Node>(d_ptr->Name, d_ptr->X, d_ptr->Y, d_ptr->cost, d_ptr->nodeType));	//define new Node
			nodes_changes = true;
		}
		else {						//Node found
			if (d_ptr->cost != N_inOld->cost || d_ptr->nodeType != N_inOld->nodeType) {  //the node changed its cost or type (start/goal/any):
				nodes_changes = true;
				N_inOld->cost = d_ptr->cost;	// = "Update cost" /*11*/
				N_inOld->nodeType = d_ptr->nodeType;
				update_rhs(N_inOld);			// = "Update Vertex" /*12*/
				updateAdjacents(N_inOld);		//should I update all the adjacent nodes' rhs??  <======================================================

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

	// once we finished updating the map:
	if (nodes_changes) {
		if (count > 1) {	//count=0+1 is the first reading
			if (vehicle_moved) {	//start node has changed	
				k_m = k_m + heuristic(ptrToGoal);
				std::cout << " => Vehicle moved (changed start node) -> new k_m=" << k_m << " .\n\n";
			}
			computeMOPaths();	/*13*/   // <=============== why??? ===================
		}

		for (auto N_ptr : NodesVect) { // fill adjacents to each node -> for sure not optimized!!!!!
			findAdjacents(N_ptr);
			//N_ptr->print_Adjacents();
		} //(can't be done in constructor because not all nodes have been registered yet)
	}
}