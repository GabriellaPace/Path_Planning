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
Sptr_toNode findNodeptr(int xx, int yy) {    // find the index of the desired node in NodesVect (matching X and Y)
	int x = xx;
	int y = yy;
	int idx = -1;  //this should raise an error if used in a vector
	auto it = find_if(Node::NodesVect.begin(), Node::NodesVect.end(), 
			   [&x, &y](const std::shared_ptr<Node>& obj) {return ((*obj).X == x && (*obj).Y == y); });
	if (it != Node::NodesVect.end()) {  //shouln't be needed, but just in case
		idx = (int) std::distance(Node::NodesVect.begin(), it);
	}
	else {
		throw " => The coordinates you are looking for are not present in the old map.\n";
	}
	return Node::NodesVect[idx];
}


Qe computeMOPaths(Qe queue) {
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

std::vector<Sptr_toNode> generateMOPaths(){
	std::vector<Sptr_toNode> a;
	return a;
}

//void DomAll(Node a, Node b) {}


////////////////////////////////   INITIALIZATIONS   ////////////////////////////////
//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
std::priority_queue<Node, std::vector<Node>, std::greater<Node>>  queue;   // filled with Nodes, NOT ptr_to_Nodes !!
bool changed_edges = false;
std::vector<Sptr_toNode> solutionPaths;
Sptr_toNode N_inOld = nullptr;
std::vector<Sptr_toNode> ChangedNodes;

/////////////////////////////////////////////////////////////////////////////////////

int main() {
	ReadMap_firstTime();
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
			try {
				N_inOld = findNodeptr((*d_ptr).X, (*d_ptr).Y);
				if ((*d_ptr).cost != (*N_inOld).cost) {   //changed edge cost!
					changed_edges = true;
					ChangedNodes.push_back(N_inOld);  //save pointers of changed ones
				}
			}
			catch(const char *err) {  // Node not found
				std::cout << err <<	"     ^ So a new node will be created.\n\n";
				Node n9((*d_ptr).Name, d_ptr->X, d_ptr->Y, (*d_ptr).cost, d_ptr->nodeType);  //define new Node
				changed_edges = true;
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
				computeMOPaths(queue); //<===============================================================================
			}
		}
		changed_edges = false;
		ChangedNodes.clear();
	//}

		printAll_g_rhs();


	//// DELETE ALL objects
	//for (auto N_ptr : Node::NodesVect) {
	//	delete &N_ptr;
	//	N_ptr = nullptr;
	//}
	std::cin.get();
}