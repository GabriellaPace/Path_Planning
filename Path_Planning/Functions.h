#pragma once
#include "ReadMap.h"


using Qe = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
//namespace pmr {
//	template <class Key, class Compare = std::less<Key>>
//	using set = std::set<Key, Compare, std::pmr::polymorphic_allocator<Key>>;
//}
using set = std::set<Node, std::less<Node>, std::pmr::polymorphic_allocator<Node> >;
	//std::pmr::polymorphic_allocator = runtime polymorphism, allows objects using polymorphic_allocator to behave as if they used different allocator types at run time despite the identical static allocator type
using Deq = std::deque<Wptr_toNode>;
using parents_map = robin_hood::unordered_map < Wptr_toNode, uint8_t >;	//only needed by debug

//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
//Qe queue;		// filled with Nodes, NOT ptr_to_Nodes !!
set queue;		// filled with Nodes, NOT ptr_to_Nodes !!

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

	//Qe q = queue;
	//std::cout << "Queue:" << std::endl;
	//Node tmp;
	//if (q.empty())
	//	std::cout << " empty!\n";
	//else {
	//	while (!q.empty()) {
	//		tmp = q.top();
	//		tmp.print_NodeKey();
	//		q.pop();
	//	}
	//}
	//std::cout << std::endl;
}

void printAll_g_rhs() {
	std::cout << "g and rhs for all current nodes:\n";
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

void print_parents(Wptr_toNode  N) {
	std::cout << "Parents of node ";	N->print_Coord();		std::cout << " :\n";
	for (parents_map::iterator it = N->parents.begin(); it != N->parents.end(); ++it) {
		std::cout << "Key (coord): ";	(it->first)->print_Coord();
		std::cout << "  -  Value: " << +(it->second) << std::endl;		//+ needed to print unsigned_int8
	}
	std::cout << std::endl;
}

///////////////////////////////////////// Functions /////////////////////////////////////////

Wptr_toNode findNodeptr(int xx, int yy) {    // find the pointer of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx; // = -1;   //<- this should raise an error if used in a vector
	//auto it = find_if(NodesVect.begin(), NodesVect.end(),
		//[&x, &y](const std::shared_ptr<Node>& obj) {return ((*obj).X == x && (*obj).Y == y); });
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
	N->key.second = nonDom_2(N->g, N->rhs);
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
		N->AdjacentsList.push_back(NodesVect[idx]);
	}
}


void findAdjacents(Wptr_toNode N) {
	int oriz, vert;
	oriz = N->X + 0;  vert = N->Y + 1;   addAdj(N, oriz, vert);
	oriz = N->X + 0;  vert = N->Y - 1;   addAdj(N, oriz, vert);
	oriz = N->X + 1;  vert = N->Y + 0;   addAdj(N, oriz, vert);
	oriz = N->X - 1;  vert = N->Y + 0;   addAdj(N, oriz, vert);
	oriz = N->X + 1;  vert = N->Y + 1;   addAdj(N, oriz, vert);
	oriz = N->X - 1;  vert = N->Y + 1;   addAdj(N, oriz, vert);
	oriz = N->X + 1;  vert = N->Y - 1;   addAdj(N, oriz, vert);
	oriz = N->X - 1;  vert = N->Y - 1;   addAdj(N, oriz, vert);
	//^ remove redundant definitions
}


std::vector<Wptr_toNode> nonDom_succs(Wptr_toNode N) {		// find non-dominated nodes among successors of the given node
	std::vector<Wptr_toNode> nonDomSuccs_tmp;
	bool nonDom_flag;
	float cC_out, cC_in;  // still considering single g and rhs -> will become vectors //cC = cumulative cost (outer/inner loop)

	for (auto adN : N->AdjacentsList) {		//for each element of AdjacentsList, we'll check if it dominated every other element in the same List
		nonDom_flag = true;
		cC_out = compute_cost(N, adN) + adN->g;

		for (auto inN : N->AdjacentsList) {		//paragona con ogni altro elemento di AdjacentsList [anche con se stesso!!] -> problema?	 
			cC_in = compute_cost(N, inN) + inN->g;
			if (domination(cC_out, cC_in) == snd_dominates) {		//it is dominated by someone-else!	//if (!nonDom_b(cC_out, cC_in)) {
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


//void update_rhs(Wptr_toNode N) {    //function UPDATE_VERTEX(u)
//	int X_start = (*ptrToStart).X;
//	int Y_start = (*ptrToStart).Y;
//
//	float current_min_rhs = N->rhs;
//	//std::shared_ptr<Node> current_pred_ptr;
//	Wptr_toNode current_pred_ptr;
//
//	for (auto A_ptr : N->AdjacentsList) {   //search among all the adjacent nodes the best one to come from
//		float d = (float)((sqrt(pow((N->X - (*A_ptr).X), 2.0f) + pow((N->Y - (*A_ptr).Y), 2.0f))) * 10);
//		//^ distance btw current node and selected adjacent one
//		float tmp_rhs = (*A_ptr).g + d;    //the rhs that this node would have if updated
//		if (tmp_rhs < current_min_rhs) {   //actually update it only if better than old one
//			current_min_rhs = tmp_rhs;
//			current_pred_ptr = A_ptr;
//		}
//	}
//	N->rhs = current_min_rhs;
//	N->parents[current_pred_ptr]; // = compute_cost(std::make_shared<Node>(*this), current_pred_ptr);
//}

void update_rhs(Wptr_toNode N) {    //function UPDATE_VERTEX(u)
	if (N->nodeType != goal) {
		//N->rhs = nonDom_succs(N);	//????????????????????

		// ^ means this (??) :

		float tmp_rhs;
		float current_min_rhs = N->rhs;
		for (auto s1 : N->AdjacentsList) {   //search among all the adjacent nodes the best one to come from
			//c = compute_cost(N, A_ptr);	//distance btw current node and selected adjacent one

			tmp_rhs = compute_cost(N, s1) + s1->g;    //the rhs that this node would have if updated
			if (domination(tmp_rhs, current_min_rhs) == fst_dominates) {   //actually update it only if better than old one
				current_min_rhs = tmp_rhs;
			}
		}
		N->rhs = current_min_rhs;
	}

	if (queue.find(*N) != queue.end()) {	// if N is in the queue, remove it
		queue.erase(*N);
	}

	if (domination(N->g, N->rhs) != areEqual) {
		calculateKey(N);


		// FOR [0,1] ENTERS HERE BUT THIS insert  DOESN'T WORK!! [0,1] IS NOT ALREADY IN THE QUEUE
		queue.insert(*N);
		//by definition doesn't allow duplicates, if inserted element is equivalent to an element already in the container, the element is not inserted, returning an iterator to this existing element (if the function returns a value)
	}

	print_queue();  //debug
}


void updateAdjacents(Wptr_toNode N) {
	for (auto A_ptr : N->AdjacentsList) {   //update each node adjacent to the modified one
		//(*A_ptr).update_rhs();
		update_rhs(A_ptr);
	}
}

/*-----------------------------------------------------------------------------------------*/



/*--------------------------------------- ROUTINES ----------------------------------------*/

void computeMOPaths() {  //function COMPUTE_MO_PATHS()	
	Node deqN_wOldKey;
	float inf = std::numeric_limits<float>::infinity();	// can't do "using"

	calculateKey(ptrToStart);

	while (!queue.empty()  &&  ( (*ptrToStart).key.second == inf || *ptrToStart < *(queue.begin()) ) ) {	// = start.key dominates the top key in the queue ( < = ordering rule, based on key)
		//Node deqN_wOldKey = queue.top();  //pick top one (deqN = de-queued Node)
		//queue.pop();					  //and then remove it
		auto queue_top = queue.begin();
		deqN_wOldKey = *queue_top;  //pick top Node in the queue (deqN = de-queued Node)
		queue.erase(queue_top);		//equivalent to "queue.pop()" -> remove dequeued node from the queue

		Wptr_toNode deqN_ptr = findNodeptr(deqN_wOldKey.X, deqN_wOldKey.Y);	 //ptr to de-queued node
		calculateKey(deqN_ptr);

		if (deqN_wOldKey < *deqN_ptr) {					 //put it back in queue with new key
			queue.insert(*deqN_ptr);
		}
		else if ((*deqN_ptr).rhs < (*deqN_ptr).g) {		 // OVERCONSISTENT
			(*deqN_ptr).g = (*deqN_ptr).rhs;
			updateAdjacents(deqN_ptr);
		}
		else if ((*deqN_ptr).rhs > (*deqN_ptr).g) {		 // UNDERCONSISTENT
			(*deqN_ptr).g = std::numeric_limits<float>::infinity();
			updateAdjacents(deqN_ptr);
			update_rhs(deqN_ptr);
		}
		else {											 // not dominant and not dominated 
			(*deqN_ptr).g = nonDom_2((*deqN_ptr).g, (*deqN_ptr).rhs);
			updateAdjacents(deqN_ptr);
		}

		calculateKey(ptrToStart); //for next loop
		printAll_g_rhs();   //print_queue();  //debug
	}
	std::cout << " => Computed MO Paths.\n\n";
}


void updateMap() {
	// initializations (and re-initializations)
	bool nodes_changes = false;
	bool vehicle_moved = false;
	Wptr_toNode N_inOld = nullptr;
	std::vector<Wptr_toNode> ChangedNodes;
	ChangedNodes.clear();


	ReadMap();	//second map  (//wait for any weight cost to change)

	for (auto d_ptr : newMap) {
		N_inOld = findNodeptr((*d_ptr).X, (*d_ptr).Y);
		if (N_inOld == nullptr) {	//Node not found
			std::cout << " => coordinates [" << d_ptr->X << "," << d_ptr->Y << "] were not in the old map, so a new node will be created.\n\n"; //debug		
			NodesVect.push_back(std::make_shared<Node>(d_ptr->Name, d_ptr->X, d_ptr->Y, d_ptr->cost, d_ptr->nodeType));	//define new Node
			nodes_changes = true;
		}
		else {						//Node found
			//the node changed its cost or type (start/goal/any):
			if (d_ptr->cost != N_inOld->cost || d_ptr->nodeType != N_inOld->nodeType) {
				nodes_changes = true;
				//ChangedNodes.push_back(N_inOld);	//save pointers of changed ones

				//N_inOld = findNodeptr(d_ptr->X, d_ptr->Y);
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
		if (vehicle_moved) {	//start node has changed	
			k_m = k_m + heuristic(ptrToGoal);
		}
		computeMOPaths();	/*13*/
	}
}