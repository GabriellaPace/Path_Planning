#pragma once
#include "Head.h"


enum NodeTypes {
	start = 0, any = 1, goal = 2
};


class Node {		// Node = state in Koeing
	using Wptr_toNode = std::shared_ptr<Node>;
	
public:
	char Name; //debug
	int X;
	int Y;
	uint8_t cost;   //read from height map (from which we will derive edge costs)
	float g;		//cost function
	float rhs;		//one step lookahead value of g
	NodeTypes nodeType;

	std::pair<float, float> key;

	std::vector<Wptr_toNode> AdjacentsList;	//all nodes adjacent to current one (const??)
	robin_hood::unordered_map < Wptr_toNode, uint8_t > parents;    //key: ptr to node, value: cumulative cost
	//robin_hood::unordered_map < Wptr_toNode, std::vector<uint8_t> > parents;    //key: ptr to node, value: cumulative cost(s)

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	Node(char name, int x, int y, uint8_t ec, NodeTypes flag)  // for actual nodes
		: Name(name), X(x), Y(y), cost(ec), nodeType(flag) {

		g = std::numeric_limits<float>::infinity();

		if (nodeType == goal) {
			//ptrToGoal = this;
			rhs = 0.0f;
		}
		else {
			rhs = std::numeric_limits<float>::infinity();
		}

		key.first = -1;
		key.second = -1;
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
	//static std::vector<std::shared_ptr<dummyNode>> newMap;	// vector of shared pointers to dummyNodes - def. in ReadMap.h
	char Name; //debug
	int X;
	int Y;
	uint8_t cost;
	NodeTypes nodeType;

/////////////////////////   Constructors   //////////////////////////
	dummyNode() {}		// for pointers etc.

	dummyNode(char name, int x, int y, uint8_t ec, NodeTypes flag) // for actual nodes
		: Name(name), X(x), Y(y), cost(ec), nodeType(flag) {}
};