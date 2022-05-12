#pragma once
#include "Head.h"


enum NodeTypes {
	start = 0, any = 1, goal = 2
};


class Node {			// Node = state in Koeing
	using Sptr_toNode = std::shared_ptr<Node>;
public:
	////// common to all istances: ////////////////////////
	static std::vector<Sptr_toNode> NodesVect;	// vector of shared pointers to Nodes - def. in ReadMap.h
	//static std::vector<std::weak_ptr<Node>> NodesVect;
	static float k_m;			   //def. in MODLite.h
	static Sptr_toNode ptrToStart; //def. in MODLite.h
	static Sptr_toNode ptrToGoal;  //def. in MODLite.h
	/*---------------------------------------------------*/
	char Name; //debug

	int X;
	int Y;
	uint8_t cost;   //read from height map (from which we will derive edge costs)
	float g;		//cost function
	float rhs;		//one step lookahead value of g
	NodeTypes nodeType;

	std::pair<float, float> key;

	std::vector<Sptr_toNode> AdjacentsList;	//all nodes adjacent to current one (const??)
	std::shared_ptr<Node> predecessor; //to remove, replaced by parents[]
	robin_hood::unordered_map < Sptr_toNode, uint8_t > parents;    //key: ptr to node, value: cumulative cost

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	Node(char name, int x, int y, uint8_t ec, NodeTypes flag) {  // for actual nodes
		Name = name;
		X = x;
		Y = y;
		cost = ec;
		nodeType = flag;

		if (nodeType == start) {
			ptrToStart = std::make_shared<Node>(*this);
		}

		g = std::numeric_limits<float>::infinity();

		if (nodeType == goal) {
			ptrToGoal = std::make_shared<Node>(*this);
			rhs = 0.0f;
		}
		else {
			rhs = std::numeric_limits<float>::infinity();
		}

		key.first = -1;
		key.second = -1;
		predecessor = nullptr;

		findAdjacents();
			print_Adjacents(); //debug
		NodesVect.push_back(std::make_shared<Node>(*this));
	}

////////////////////////////   sorting criteria for queue   /////////////////////////
	bool operator > (const Node &N2) const {
		if (key.first > N2.key.first)
			return true;
		else if (key.first < N2.key.first)
			return false;
		else { // key.first == N2.key.first
			if (key.second > N2.key.second)
				return true;
			else // (key.second <= N2.key.second)
				return false;
		}
	}

	bool operator < (const Node &N2) const {
		if (key.first < N2.key.first)
			return true;
		else if (key.first > N2.key.first)
			return false;
		else { // key.first == N2.key.first
			if (key.second < N2.key.second)
				return true;
			else // (key.second >= N2.key.second)
				return false;
		}
	}

/////////////////////////////////////   Methods   //////////////////////////////////
	void calculateKey() {
		key.second = nonDom_2(g, rhs);
		key.first = key.second + heuristic() + k_m;
	}

	int heuristic() {		// shortest aereal path (ignoring the grid)
		int X_start = (*ptrToStart).X;
		int Y_start = (*ptrToStart).Y;
		
		int h = (int)((sqrt(pow((X - X_start), 2.0f) + pow((Y - Y_start), 2.0f)))*10);  //pow(base, power)
		//std::cout << "Heuristic of node [" << X << "," << Y << "] : " << h ;
		//std::cout << "   (-> wrt start node with coord: [" << X_start << "," << Y_start << "] )" << std::endl << std::endl;
		return h;
	}
/*--------------------------------------------------------------------------------*/
	void findAdjacents() {
		int oriz, vert;
		oriz = X + 0;  vert = Y + 1;   addAdj(oriz, vert);
		oriz = X + 0;  vert = Y - 1;   addAdj(oriz, vert);
		oriz = X + 1;  vert = Y + 0;   addAdj(oriz, vert);
		oriz = X - 1;  vert = Y + 0;   addAdj(oriz, vert);
		oriz = X + 1;  vert = Y + 1;   addAdj(oriz, vert);
		oriz = X - 1;  vert = Y + 1;   addAdj(oriz, vert);
		oriz = X + 1;  vert = Y - 1;   addAdj(oriz, vert);
		oriz = X - 1;  vert = Y - 1;   addAdj(oriz, vert);
		//^ remove redundant definitions
	}

	void addAdj(int oriz, int vert) {
		auto it = find_if(NodesVect.begin(), NodesVect.end(), [&oriz, &vert](const Sptr_toNode& obj) {return ((*obj).X == oriz && (*obj).Y == vert); });
		if (it != NodesVect.end()) {
			auto idx = std::distance(NodesVect.begin(), it);
			AdjacentsList.push_back(NodesVect[idx]);
		}
	}
/*--------------------------------------------------------------------------------*/
	void updateAdjacents() {
		for (auto A_ptr : AdjacentsList) {   //update each node adjacent to the modified one
			(*A_ptr).update_rhs();
		}
	}

	void update_rhs() {    //function UPDATE_VERTEX(u)
		int X_start = (*ptrToStart).X;
		int Y_start = (*ptrToStart).Y;

		float current_min_rhs = rhs;
		std::shared_ptr<Node> current_pred_ptr;

		for (auto A_ptr : AdjacentsList) {   //search among all the adjacent nodes the best one to come from
			float d = (float)((sqrt(pow((X - (*A_ptr).X), 2.0f) + pow((Y - (*A_ptr).Y), 2.0f))) * 10);
				//^ distance btw current node and selected adjacent one
			float tmp_rhs = (*A_ptr).g + d;    //the rhs that this node would have if updated
			if (tmp_rhs < current_min_rhs) {   //actually update it only if better than old one
				current_min_rhs = tmp_rhs;
				current_pred_ptr = A_ptr;
			}
		}
		rhs = current_min_rhs;
		predecessor = current_pred_ptr; //to remove
		parents[current_pred_ptr];// = compute_cost(std::make_shared<Node>(*this), current_pred_ptr);
	}

//////////////////////////////////   debug Methods   ////////////////////////////////
	void print_Coord() {
		std::cout << "[" << X << "," << Y << "]";
	}

	void print_NodeKey() {
		std::cout << "( "<< key.first <<" , "<< key.second <<" ) -> [" << X << "," << Y << "]" << std::endl;
	}

	void print_g_rhs() {
		std::cout << "[" << X << "," << Y << "] -> g=" << g << " , rhs=" << rhs << std::endl;
	}

	void print_Adjacents() {
		std::cout << "Adjacents to node ";
		print_Coord();
		std::cout << " : ";
		
		for (auto A_ptr : AdjacentsList) {
			(*A_ptr).print_Coord();
			std::cout << "  ";
		}
		std::cout << std::endl;
	}
};




class dummyNode {
public:
	static std::vector<std::shared_ptr<dummyNode>> newMap;	// vector of shared pointers to dummyNodes - def. in ReadMap.h
	char Name; //debug
	int X;
	int Y;
	uint8_t cost;
	NodeTypes nodeType;

/////////////////////////   Constructors   //////////////////////////
	dummyNode() {}		// for pointers etc.

	dummyNode(char name, int x, int y, uint8_t ec, NodeTypes flag) {  // for actual nodes
		Name = name;
		X = x;
		Y = y;
		nodeType = flag;
		cost = ec;

		newMap.push_back(std::make_shared<dummyNode>(*this));
	}
};