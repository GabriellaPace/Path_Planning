#pragma once
#include "Functions.h"


enum NodeTypes {
	start = 0, any = 1, goal = 2
};


class Node {			// Node = state in Koeing
	using Sptr_toNode = std::shared_ptr<Node>;
public:
	////// common to all istances: ////////////////////////
	static std::vector<std::shared_ptr<Node>> NodesVect;	// vector of shared pointers to Nodes - def. in ReadMap.h
	//static std::vector<std::weak_ptr<Node>> NodesVect;
	static float k_m;			   //def. in MODLite.h
	static Sptr_toNode ptrToStart; //def. in MODLite.h
	/*---------------------------------------------------*/
	char Name; //debug

	int X;
	int Y;

	NodeTypes nodeType;

	float g;
	float rhs;
	//float cost;

	std::pair<float, float> key;

	std::shared_ptr<Node> predecessor;
	std::vector<std::shared_ptr<Node>> AdjacentsList;	//all nodes adjacent to current one (const??)

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	Node(char name, int x, int y, NodeTypes flag) {  // for actual nodes
		Name = name;
		X = x;
		Y = y;

		nodeType = flag;

		if (nodeType == start) {
			ptrToStart = std::make_shared<Node>(*this);
		}

		g = std::numeric_limits<float>::infinity();

		if (nodeType == goal) {
			rhs = 0.0f;
		}
		else {
			rhs = std::numeric_limits<float>::infinity();
		}

		key.first = -1;
		key.second = -1;
		predecessor = nullptr;

		NodesVect.push_back(std::make_shared<Node>(*this));
	}

////////////////////////////   sorting criteria for queue   /////////////////////////
	bool operator > (const Node &N2) const {
		//bool result;
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

/////////////////////////////////////   Methods   //////////////////////////////////
	void calculateKey() {
		key.second = nonDom(g, rhs);
		key.first = key.second + heuristic() + k_m;
	}

	int heuristic() {		// shortest aereal path (ignoring the grid)
		int X_start = (*ptrToStart).X;
		int Y_start = (*ptrToStart).Y;
		
		int h = (int)((sqrt(pow((X - X_start), 2.0f) + pow((Y - Y_start), 2.0f)))*10);  //pow(base, power)
		std::cout << "Heuristic of node [" << X << "," << Y << "] : " << h ;
		std::cout << "   (-> wrt start node with coord: [" << X_start << "," << Y_start << "] )" << std::endl << std::endl;
		return h;
	}

	void findAdjacents() {
		int oriz, vert;
		oriz = X + 0;  vert = Y + 1;
		addAdj(oriz, vert);
		oriz = X + 0;  vert = Y - 1;
		addAdj(oriz, vert);
		oriz = X + 1;  vert = Y + 0;
		addAdj(oriz, vert);
		oriz = X - 1;  vert = Y + 0;
		addAdj(oriz, vert);
		oriz = X + 1;  vert = Y + 1;
		addAdj(oriz, vert);
		oriz = X - 1;  vert = Y + 1;
		addAdj(oriz, vert);
		oriz = X + 1;  vert = Y - 1;
		addAdj(oriz, vert);
		oriz = X - 1;  vert = Y - 1;
		addAdj(oriz, vert);
	}

	void addAdj(int oriz, int vert) {
		auto it = find_if(NodesVect.begin(), NodesVect.end(), [&oriz, &vert](const Sptr_toNode& obj) {return ((*obj).X == oriz && (*obj).Y == vert); });
		if (it != NodesVect.end()) {
			auto idx = std::distance(NodesVect.begin(), it);
			AdjacentsList.push_back(NodesVect[idx]);
		}
	}

	//void updateAdjacents() {
	//	for (auto &N : AdjacentsList) {
	//		update_rhs();
	//	}
	//}

	//void update_rhs() {
	//	min_rhs = rhs;
	//	for (auto A : AdjacentsList) {
	//		float d = (float)((sqrt(pow((X - X_start), 2.0f) + pow((Y - Y_start), 2.0f))) * 10);	//distance btw current node and selected adjacent node
	//		float tmp_rhs = A.g + d;
	//		if (tmp_rhs < min_rhs) {
	//			min_rhs = tmp_rhs;
	//			void* pred_ptr = *A;
	//		}
	//	}
	//	rhs = min_rhs;
	//	predecessor = pred_ptr;
	//}

//////////////////////////////////   debug Methods   ////////////////////////////////
	void print_Coord() {
		std::cout << "[" << X << "," << Y << "]";
	}

	void print_NodeKey() {
		std::cout << "( "<< key.first <<" , "<< key.second <<" ) -> [" << X << "," << Y << "]" << std::endl;
	}

	void print_Adjacents() {
		std::cout << "Adjacents to node ";
		print_Coord();
		std::cout << " : ";
		
		for (auto A_ptr : AdjacentsList) {
			(*A_ptr).print_Coord();
			std::cout << "  ";
		}
		std::cout << std::endl << std::endl;
	}
};