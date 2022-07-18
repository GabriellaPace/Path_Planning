#pragma once
#include "ReadMap.h"

clock_t startTime, startTime_whole;	//debug

using set = std::set<Node, std::less<Node>, std::pmr::polymorphic_allocator<Node> >;	//by definition doesn't allow duplicates
	//std::pmr::polymorphic_allocator = allows objects to behave as if they used different allocator types despite the identical static allocator type
using Deq = std::deque<Sptr_toNode>;	//for expanding_states (unordered queue)
using parents_map = robin_hood::unordered_map < Sptr_toNode, int >;

set queue;	//filled with Nodes, NOT ptr_to_Nodes !!

Sptr_toNode  ptrToStart = nullptr, ptrToGoal = nullptr;
float k_m = 0;

////////////////////////////////////// Debug functions //////////////////////////////////////

//void print_queue() {
//	std::cout << "Queue:" << std::endl;
//	if (queue.empty())
//		std::cout << " empty!\n";
//	else {
//		for (auto N : queue) {
//			N.print_NodeKey();
//		}
//	}
//	std::cout << std::endl;
//}

//void printAll_g_rhs() {
//	std::cout << "\ng and rhs for all current nodes:\n";
//	for (auto [N_coord, N_ptr] : allNodes) {
//		(*N_ptr).print_g_rhs();
//	}
//	std::cout << std::endl;
//}

//void print_intVect(std::vector<int> vect) {
//	for (int i = 0; i < vect.size(); ++i) {
//		std::cout << +vect[i] << std::endl;		//+ needed to print unsigned_int8
//	}
//	std::cout << std::endl;
//}

//void print_parents(Sptr_toNode N) {
//	std::cout << "Parent of node = "; N->print_Coord();		std::cout << " :\n";
//	for (auto&[par_ptr, par_cost] : N->parents) {
//		par_ptr->print_Coord();   std::cout << "  :  " << +par_cost << std::endl;
//	}
//	std::cout << std::endl;
//}

void print_solution(std::vector<Sptr_toNode> solution_vect) {
	std::cout << "SOLUTION PATH for map {" << map_count << "} optimized for g[1]:\n";
	for (auto ptr : solution_vect){
		ptr->print_Coord();
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void save_solution_img(std::vector<Sptr_toNode> solution_vect) {
	#ifdef BLUR_MAP
		cv::Mat img_mat = cv::imread((img_path + std::to_string(map_count) + "_image_SOL.bmp").c_str(), cv::IMREAD_GRAYSCALE);		//ReadMap() focus
	#else
		cv::Mat img_mat = cv::imread((img_path + std::to_string(map_count) + "_gradient.bmp").c_str(), cv::IMREAD_GRAYSCALE);		//ReadMap() x100
	#endif // BLUR_MAP	
	
	for (auto ptr : solution_vect) {
		img_mat.at<uchar>(ptr->XY.first, ptr->XY.second) = 0;
	}


	#ifdef BLUR_MAP
		cv::imwrite((img_path + std::to_string(map_count) + "_image_SOL.bmp").c_str(), img_mat);			//ReadMap() focus
	#else
		cv::imwrite((img_path + std::to_string(map_count) + "_gradient_SOL.bmp").c_str(), img_mat);		//ReadMap() x100
	#endif // BLUR_MAP	

	for (auto [val, ptr] : allNodes) {	//image which higlights the expanded nodes
		if(ptr->expanded)
			img_mat.at<uchar>(ptr->XY.first, ptr->XY.second) = 255;	 }
	cv::imwrite((img_path + "image_exp.bmp").c_str(), img_mat);
}

///////////////////////////////////////// Functions /////////////////////////////////////////

flt_vect vector_sum(flt_vect a, flt_vect b) {
	if (a.empty()) return b;
	if (b.empty()) return a;

	flt_vect result;
	for (int i = 0; i < std::max(a.size(), b.size()); ++i) {	//std::max() needed because one of them might be empty (otherwise they have the same size!!
		result.push_back(a[i] + b[i]);
	}
	return result;
}


float heuristic(Sptr_toNode N) {		// shortest aereal path (ignoring the grid)
	int X_start = (*ptrToStart).XY.first;
	int Y_start = (*ptrToStart).XY.second;

	float h = (float)((sqrt(pow((N->XY.first - X_start), 2.0f) + pow((N->XY.second - Y_start), 2.0f))) * 10);  //pow(base, power)
	return h;
}


void calculateKey(Sptr_toNode N) {
	N->key.second = nonDom(N->g, N->rhs).front();												//@@@ NOT DESCRIBED IN PAPER  ...[0];
	N->key.first = N->key.second + heuristic(N) + k_m;
}


flt_vect compute_cost(Sptr_toNode n1, Sptr_toNode n2) {	// edge-cost derived from node-costs
	//return std::max(n1->cost, n2->cost);	//<- as done for Theta* Planner in Nav2

	if (domination(n1->cost, n2->cost) == snd_dominates || domination(n1->cost, n2->cost) == snd_completely_dominates)	// dominate = min
		return n1->cost;	//max of the two  <-  as done for Theta* Planner in Nav2
	else
		return n2->cost;
	// if areEqual or nonDomination, return n2
}

/*------------------------------------- Nodes updates -------------------------------------*/

void addAdj(Sptr_toNode N, std::pair<int, int> coord) {
	if (allNodes.find(coord) != allNodes.end()){
		N->AdjacentsVect.push_back(allNodes[coord]);
	}
}


void findAdjacents(Sptr_toNode N) {
	addAdj(N, std::pair<int, int>(N->XY.first    ,  N->XY.second + 1) );
	addAdj(N, std::pair<int, int>(N->XY.first	 ,  N->XY.second - 1) );
	addAdj(N, std::pair<int, int>(N->XY.first + 1,  N->XY.second    ) );
	addAdj(N, std::pair<int, int>(N->XY.first - 1,  N->XY.second    ) );
	addAdj(N, std::pair<int, int>(N->XY.first + 1,  N->XY.second + 1) );
	addAdj(N, std::pair<int, int>(N->XY.first - 1,  N->XY.second + 1) );
	addAdj(N, std::pair<int, int>(N->XY.first + 1,  N->XY.second - 1) );
	addAdj(N, std::pair<int, int>(N->XY.first - 1,  N->XY.second - 1) );
}


struct NDS_struct {
	std::vector<Sptr_toNode> NDS_succs;
	flt_vect NDS_rhs;
};

struct NDS_struct nonDom_succs(Sptr_toNode N) {			// find non-dominated successors, wrt multiobjective c+g
														//nonDomSuccs = nonDom_[s' in succ(Ns)](sum(c(Ns, s’), g(s’)) 
	NDS_struct NDS_sol;
	bool nonDom_flag;
	//float cC_out, cC_in;  // still considering single g and rhs -> will become vectors //cC = cumulative cost (outer/inner loop)
	std::vector<float> cC_out, cC_in;  //cC = cumulative cost (outer/inner loop)

	for (auto adNi : N->AdjacentsVect) {		//for each element of AdjacentsVect, we'll check if it dominated every other element in the same List
		nonDom_flag = true;
		cC_out = vector_sum( compute_cost(N, adNi), adNi->g );

		for (auto adNj : N->AdjacentsVect) {		//paragona con ogni altro elemento di AdjacentsVect [anche con se stesso!!]
			cC_in = vector_sum( compute_cost(N, adNj), adNj->g );

			if (domination(cC_out, cC_in) == snd_dominates	||  domination(cC_out, cC_in) == snd_completely_dominates) {	//it is dominated by someone-else!
				nonDom_flag = false;
				break;
			}
		}

		if (nonDom_flag  &&  find(NDS_sol.NDS_succs.begin(), NDS_sol.NDS_succs.end(), adNi) == NDS_sol.NDS_succs.end()) {	// to avoid duplicates
			NDS_sol.NDS_succs.push_back(adNi);
			NDS_sol.NDS_rhs = cC_out;	//c(N,s1) + g(s1)
		}
	}

	//if (NDS_sol.NDS_succs.size() > 3)	std::cout << "mh ";	//@@
	return NDS_sol;
}


void update_rhs(Sptr_toNode N) {    //function UPDATE_VERTEX(u)
	N->expanded = true;

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


void updateAdjacents(Sptr_toNode N) {
	for (auto A_ptr : N->AdjacentsVect) {   //update each node adjacent to the modified one
		update_rhs(A_ptr);
	}
}


bool start_doesNot_dominate(const Node N) {	// like < operator of Node (dominant = min)
	if (ptrToStart->key.first < N.key.first)
		return false;
	else if (ptrToStart->key.first > N.key.first)
		return true;
	else { // ptrToStart->key.first == N->key.first
		if (ptrToStart->key.second < N.key.second)
			return false;
		else if (ptrToStart->key.second > N.key.second)
			return true;
		else { // k1==k1 & k2==k2
			// to avoid premature termination in computeMOPaths() -> if Start has same key of top-node, execute! (so appears >)
			return true;
		}
	}
}

/*--------------------------------------- ROUTINES ----------------------------------------*/

void computeMOPaths() {  //function COMPUTE_MO_PATHS()
	startTime = clock(); //debug

	Node deqN_wOldKey;
	Sptr_toNode deqN_ptr;
	float inf = std::numeric_limits<float>::infinity();	// can't do "using"

	calculateKey(ptrToStart);

	while ( !queue.empty()	&&	start_doesNot_dominate(*(queue.begin())) ) {	//termination criteria = start.key dominates the top key in the queue	
		auto queue_top = queue.begin();
		deqN_wOldKey = *queue_top;			//pick top Node in the queue (deqN = de-queued Node)
		queue.erase(queue_top);				//equivalent to "queue.pop()" -> remove dequeued node from the queue

		deqN_ptr = allNodes[deqN_wOldKey.XY];	 //ptr to de-queued node

		calculateKey(deqN_ptr);

		if (deqN_wOldKey < *deqN_ptr) {		//put it back in queue with new key
			queue.insert(*deqN_ptr);		//§§ seems faster without it (??)
		}
		else if (domination(deqN_ptr->rhs, deqN_ptr->g) == fst_completely_dominates) {		 // OVERCONSISTENT (rhs<g)
			deqN_ptr->g = deqN_ptr->rhs;
			updateAdjacents(deqN_ptr);
		}
		else if (domination(deqN_ptr->rhs, deqN_ptr->g) == snd_completely_dominates) {		// UNDERCONSISTENT (rhs>g)
			for(auto deqN_ptr_gi : deqN_ptr->g)		deqN_ptr_gi = std::numeric_limits<float>::infinity();
			update_rhs(deqN_ptr);
			updateAdjacents(deqN_ptr);
		}
		else {											 // not dominant and not dominated 
			deqN_ptr->g = nonDom(deqN_ptr->g, deqN_ptr->rhs);
			updateAdjacents(deqN_ptr);
		}

		calculateKey(ptrToStart); //for next loop (needed??)
	}
	std::cout << "  - Computed MO Paths, done in  " << double(clock() - startTime) / (double)CLOCKS_PER_SEC << " s." << std::endl;
}


std::vector<Sptr_toNode> generateMOPaths() {	//function GENERATE_MO_PATHS()  
												//expanding a state = observe the domination between g and rhs
												//Ns = node to expand, s1 = nondominated successor of Ns  (s1 = s’ ,  s2 = s’’)
	// DECLARATIONS
	startTime = clock(); //debug

	Deq expandingStates;	// (de)queue of (ptr to) nodes which adjacents should be updated = to expand (FIFO)
	std::vector<Sptr_toNode> nonDomSuccs;
	std::vector<Sptr_toNode> solutionPaths;
	//int cumulativeCs;
	flt_vect cumulativeCs;
	Sptr_toNode Ns;

/*-- FIRST phase (from start to goal) -----------------------------------------------------------*/
	expandingStates.push_back(ptrToStart);

	while (!expandingStates.empty()) {
		Ns = expandingStates.front();	//Java: poll() returns the element at the head of the Queue [returns null if the Queue is empty]
		expandingStates.pop_front();

		nonDomSuccs = nonDom_succs(Ns).NDS_succs;	// find non-dominated successors, wrt multiobjective c+g	

		for (auto s1 : nonDomSuccs) {
			cumulativeCs.clear();	//re-initialization

			if (Ns->parents.empty()) {					// if Ns doesn't have any parent (only iff Ns=Start): for sure s' does not have any parent as well!
				s1->parents[Ns] = compute_cost(Ns, s1);	// ^ so Ns is added as a parent of s' with corresponding cost c(s, s').
			}
			else {										// if Ns does have predefined parents
				/*10*/	// cumulativeC = sum( c(Ns,s1), Ns.parents().values() ) 
				for (auto&[s1_ptr, s1_cost] : Ns->parents) {
					cumulativeCs = vector_sum(cumulativeCs, s1_cost);	//cumulativeCs += s1_cost;	//"aggregated" cost??
				}
				cumulativeCs = vector_sum(cumulativeCs, compute_cost(Ns, s1));	//cumulativeCs += compute_cost(Ns, s1);

				/*11-12*/
				if (s1->parents.empty()) {
					s1->parents[Ns] = cumulativeCs;		//s1.parents().put(s, cumulativeC);
				}
				else {
					for (auto&[s2_ptr, s2_cost] : s1->parents) {		//for (auto s'' : s'.parents() ) {
						if (domination(s2_cost, cumulativeCs) == areEqual || domination(s2_cost, cumulativeCs) == fst_completely_dominates) {
							break;
						}
						else if (domination(s2_cost, cumulativeCs) == snd_completely_dominates) {
							s1->parents.erase(s2_ptr);
							s1->parents[Ns] = cumulativeCs;		//s1.parents().put(s, cumulativeC);
						}
						else {	//never occurs for single-obj
							if (obj == 1)	std::cout << " [!!] SOMETING'S WRONG";

								//	flt_vect cC = cumulativeCs;
								//	flt_vect eC = s1->parents[s2_ptr];	//=s2_cost
								//
								//	if (domination(cC, eC) == areEqual || domination(cC, eC) == snd_dominates) {
								//		cumulativeCs.clear();	// §§ 
								//		break;
								//	}
								//	else if (domination(cC, eC) == fst_dominates) {
								//		s1->parents.erase(s2_ptr);
								//		break;
								//	}
								//if (s1->parents[s2_ptr].empty()) { //how can this happen?????? §§
								//	s1->parents.erase(s2_ptr);
								//}

							for (auto cC : cumulativeCs) {	// = for each type of cost (?) §§
								for (auto eC : s1->parents[s2_ptr]) {
									if (single_domination(cC, eC) == areEqual || single_domination(cC, eC) == snd_dominates) {
										cumulativeCs.erase( std::remove(cumulativeCs.begin(), cumulativeCs.end(), cC ),  cumulativeCs.end()); //§§ 
										break;
									}
									else if (single_domination(cC, eC) == fst_dominates) {
										cumulativeCs.erase( std::remove(s1->parents[s2_ptr].begin(), s1->parents[s2_ptr].end(), eC), cumulativeCs.end());
										break;
									}
								}		
								if (s1->parents[s2_ptr].empty()) { //how can this happen?????? §§
									s1->parents.erase(s2_ptr);
								}
							}

							
							if ( ! cumulativeCs.empty()) {
								s1->parents[Ns] = cumulativeCs;		//s1.parents().put(s, cumulativeCs);
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
	}


/*-- SECOND phase (from goal to start) ----------------------------------------------------------*/
	//solutionPaths = construct paths recursively traversing parents;
	flt_vect min_g1; //can't be <int>, otherwise the inf is too low
	for (int i=0; i<obj; ++i)	 min_g1.push_back(std::numeric_limits<float>::infinity());
	Sptr_toNode parent_toPush = NULL;

	Sptr_toNode N = ptrToGoal;	//might change it to Ns (to avoid useless new defintition)
	solutionPaths.clear();
	solutionPaths.push_back(N);


	while (N->nodeType != start) {
		if (N->parents.empty()) {	//=failed to gnerate a complete path
			solutionPaths.clear();
			return solutionPaths;
		}

		// (*) : repeat same for the other entries of the cost vector -> to obtain a path which each optimize one of the costs
		for(auto min_g1i : min_g1)	 min_g1i = std::numeric_limits<float>::infinity();
		for (auto&[par_ptr, par_cost] : N->parents) {
			if ( domination(par_cost, min_g1)==fst_dominates || domination(par_cost, min_g1) == fst_completely_dominates ) {	//if (par_cost < min_g1) {
				min_g1 = par_cost;
				parent_toPush = par_ptr;	//should always do this at least once
			}
		}
		solutionPaths.push_back(parent_toPush);
		// ^ (*)

		if (parent_toPush == NULL)	//debug -> should NEVER happen
			std::cout << "  ERROR in parent_toPush\n";

		N = parent_toPush;	//for next iteration
	}
	std::reverse(solutionPaths.begin(), solutionPaths.end());	//to have it from start to goal

	std::cout << "  - Generated MO Paths, done in  " << double(clock() - startTime) / (double)CLOCKS_PER_SEC << " s." << std::endl;	//debug
	return solutionPaths;
}


void updateMap() {
	startTime = clock(); //debug

	ReadMap();	// to add -> wait for any weight cost to change

	if (successful_read) {
		// initializations
		bool nodes_changes = false, vehicle_moved = false;
		Sptr_toNode N_inOld = nullptr;
		std::vector<Sptr_toNode> newNodes;	newNodes.clear();	//newNodes contains the node changed from prevuois read
																//newMap contains all the nodes read in the current map

		for (auto [d_coord, d_ptr] : newMap) {	//d_coord = XY

			N_inOld = allNodes[d_coord];

			if (N_inOld == nullptr) {	//Node not found
				//std::cout << " => coordinates [" << d_ptr->X << "," << d_ptr->Y << "] were not in the old map, so a new node will be created.\n"; //debug	
				allNodes[d_coord] = std::make_shared<Node>(d_coord, d_ptr->cost, d_ptr->nodeType);	//define new Node
				newNodes.push_back(allNodes[d_coord]);	// used later to update adjacents (should always find it, it was just created!)
				nodes_changes = true;
			}
			else {						//Node found
				if (d_ptr->cost != N_inOld->cost || d_ptr->nodeType != N_inOld->nodeType) {  //the node changed its cost or type (start/goal/any):
					if (d_ptr->nodeType != N_inOld->nodeType   &&   d_ptr->nodeType == start) {
						vehicle_moved = true;
					}
					nodes_changes = true;
					N_inOld->cost = d_ptr->cost;	// = "Update cost" /*11*/
					N_inOld->nodeType = d_ptr->nodeType;
					update_rhs(N_inOld);			// = "Update Vertex" /*12*/
					N_inOld->parents.clear();					//NOT OPTIMIZED (1)	@
					for (auto adj : N_inOld->AdjacentsVect) {	//NOT OPTIMIZED (1) @
						adj->parents.clear();					//NOT OPTIMIZED (1) @
					}
				}
				//else: Node found but there were no modifications to it
			}

			if (nodes_changes) {
				if (d_ptr->nodeType == start) {
					ptrToStart = allNodes[d_coord];
				}
				if (d_ptr->nodeType == goal) {
					ptrToGoal = allNodes[d_coord];
				}
			}
		}

		// once we finished updating the map:
		if (nodes_changes) {
			for (auto newN : newNodes) {	//fill adjacents to each node
				findAdjacents(newN);
				
				if (map_count > 0) {	//map_count=0 is the first reading
					for (auto ad_newN : newN->AdjacentsVect)
						addAdj(ad_newN, newN->XY);	//adding the new node as adjacents to his adjacents
				}
			} //(can't be done in constructor because not all nodes have been registered yet)
			//the nodes are only added, if a node becomes unavaliable it remains in the list but with cost = inf


			if (map_count > 0) {		//map_count=0 is the first reading
				if (vehicle_moved) {	//start node has changed	
					k_m += heuristic(ptrToGoal);
					std::cout << " => Vehicle moved (changed start node) -> new k_m=" << k_m << " .\n\n";
				}

				std::cout << "  - UpdateMap() done in  " << double(clock() - startTime) / (double)CLOCKS_PER_SEC << " s." << std::endl; //debug
				computeMOPaths();	/*13*/
			}
		}

	}

	newMap.clear();


	if (map_count == 0) { //debug
		std::cout << "  - Updated Map, done in  " << double(clock() - startTime) / (double)CLOCKS_PER_SEC << " s." << std::endl; //debug
	}
}