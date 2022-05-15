#pragma once
#include "ReadMap.h"


using Qe = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
using Deq = std::deque<Wptr_toNode>;

// definition of Node's static variables
//float Node::k_m = 0.0f;
//std::vector<std::shared_ptr<Node>> Node::NodesVect;			//std::vector<std::weak_ptr<Node>> Node::NodesVect;
//std::vector<std::shared_ptr<dummyNode>> newMap;
//Sptr_toNode  Node::ptrToStart = nullptr;
//Sptr_toNode  Node::ptrToGoal = nullptr;
	//declatation and definition

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

/*----------------------------------  Functions  ----------------------------------*/
//Sptr_toNode findNodeptr(int xx, int yy) {    // find the pointer of the desired node in NodesVect (matching X and Y)
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

////////////////////////////////////////// METHODS //////////////////////////////////////////

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

/*--------------------------------------------------------------------------------*/
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


void update_rhs(Wptr_toNode N) {    //function UPDATE_VERTEX(u)
	int X_start = (*ptrToStart).X;
	int Y_start = (*ptrToStart).Y;

	float current_min_rhs = N->rhs;
	//std::shared_ptr<Node> current_pred_ptr;
	Wptr_toNode current_pred_ptr;

	for (auto A_ptr : N->AdjacentsList) {   //search among all the adjacent nodes the best one to come from
		float d = (float)((sqrt(pow((N->X - (*A_ptr).X), 2.0f) + pow((N->Y - (*A_ptr).Y), 2.0f))) * 10);
		//^ distance btw current node and selected adjacent one
		float tmp_rhs = (*A_ptr).g + d;    //the rhs that this node would have if updated
		if (tmp_rhs < current_min_rhs) {   //actually update it only if better than old one
			current_min_rhs = tmp_rhs;
			current_pred_ptr = A_ptr;
		}
	}
	N->rhs = current_min_rhs;
	N->predecessor = current_pred_ptr; //to remove
	N->parents[current_pred_ptr];// = compute_cost(std::make_shared<Node>(*this), current_pred_ptr);
}


void updateAdjacents(Wptr_toNode N) {
	for (auto A_ptr : N->AdjacentsList) {   //update each node adjacent to the modified one
		//(*A_ptr).update_rhs();
		update_rhs(A_ptr);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////

//uint8_t compute_cost(Sptr_toNode n1, Sptr_toNode n2) {	// edge-cost derived from node-costs
uint8_t compute_cost(Wptr_toNode n1, Wptr_toNode n2) {	// edge-cost derived from node-costs
	return std::max(n1->cost, n2->cost);	//<- as done for Theta* Planner in Nav2
}



std::vector<Wptr_toNode> nonDom_succs(Wptr_toNode N) {		// find non-dominated nodes among successors of the given node
	//std::vector<Sptr_toNode> nonDomSuccs_tmp;
	std::vector<Wptr_toNode> nonDomSuccs_tmp;
	bool nonDom_flag;
	float cC_out, cC_in;  // still considering single g and rhs -> will become vectors //cC = cumulative cost (outer/inner loop)

	for(auto adN : N->AdjacentsList) {		//per ogni elemento di AdjacentsList
		nonDom_flag = true;
		cC_out = compute_cost(N, adN) + adN->g;
		
		for (auto inN : N->AdjacentsList) {		//paragona con ogni altro elemento di AdjacentsList [anche con se stesso!! -> problema?	 
			cC_in = compute_cost(N, inN) + inN->g;
			//if (!nonDom_b(cC_out, cC_in)) {
			if (domination(cC_out, cC_in) == snd_dominates) {		//it is dominated by someone-else!
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


///////////////////////////////////////// ROUTINES //////////////////////////////////////////


Qe computeMOPaths(Qe queue) {  //function COMPUTE_MO_PATHS()	
	//(*(ptrToStart)).calculateKey();
	calculateKey(ptrToStart);
	while (!queue.empty() && *ptrToStart > queue.top()) {	// = start.key dominates the top key in the queue
		Node deqN_wOldKey = queue.top();  //pick top one (deqN = de-queued Node)
		queue.pop();					  //and then remove it

		//Sptr_toNode deqN_ptr = findNodeptr(deqN_wOldKey.X, deqN_wOldKey.Y);	 //ptr to de-queued node
		Wptr_toNode deqN_ptr = findNodeptr(deqN_wOldKey.X, deqN_wOldKey.Y);	 //ptr to de-queued node
		//(*deqN_ptr).calculateKey();
		calculateKey(deqN_ptr);

		if (deqN_wOldKey < *deqN_ptr) {					 //put it back in queue with new key
			queue.push(*deqN_ptr);
		}
		else if ((*deqN_ptr).rhs < (*deqN_ptr).g) {		 // OVERCONSISTENT
			(*deqN_ptr).g = (*deqN_ptr).rhs;
			//(*deqN_ptr).updateAdjacents();
			updateAdjacents(deqN_ptr);
		}
		else if ((*deqN_ptr).rhs > (*deqN_ptr).g) {		 // UNDERCONSISTENT
			(*deqN_ptr).g = std::numeric_limits<float>::infinity();
			//(*deqN_ptr).updateAdjacents();
			updateAdjacents(deqN_ptr);
			//(*deqN_ptr).update_rhs();
			update_rhs(deqN_ptr);
		}
		else {											 // not dominant and not dominated 
			(*deqN_ptr).g = nonDom_2((*deqN_ptr).g, (*deqN_ptr).rhs);
			//(*deqN_ptr).updateAdjacents();
			updateAdjacents(deqN_ptr);
		}

		//(*(ptrToStart)).calculateKey(); //for next loop
		calculateKey(ptrToStart); //for next loop
	}
	std::cout << " => Computed MO Paths.\n\n";
	return queue;
}