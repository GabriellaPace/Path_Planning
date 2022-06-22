#pragma once
#include "Head.h"
#define OPTIMIZE //tentativo pre migliorare performance

enum NodeTypes {
	start = 0, any = 1, goal = 2
};


class Node {		// Node = state in Koeing
	using Sptr_toNode = std::shared_ptr<Node>;
	
public:
	//char Name; //debug
	int X, Y;
	int cost;   //read from height map (from which we will derive edge costs)
	float g, rhs;	//g = cost function,	rhs = one step lookahead value of g
	//std::vector<float> g, rhs;
	NodeTypes nodeType;

	std::pair<float, float> key;

	std::vector<Sptr_toNode> AdjacentsVect;	//all nodes adjacent to current one (const??)
	#ifdef OPTIMIZE
		bool AdjComputed = false;
	#endif //OPTIMIZE

	robin_hood::unordered_map < Sptr_toNode, int > parents;    //key: ptr to node, value: cumulative cost
	//robin_hood::unordered_map < Sptr_toNode, std::vector<int> > parents;    //key: ptr to node, value: cumulative cost(s)

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	Node(int x, int y, int ec, NodeTypes flag)  // for actual nodes
		: X(x), Y(y), cost(ec), nodeType(flag) {

		g = std::numeric_limits<float>::infinity();

		if (nodeType == goal) {
			rhs = 0.0f;
		}
		else {
			rhs = std::numeric_limits<float>::infinity();
		}

		key.first = -1;
		key.second = -1;
	}

////////////////////////////   sorting criteria for queue   /////////////////////////
	bool operator < (const Node &N2) const {
		if (key.first < N2.key.first)
			return true;
		else if (key.first > N2.key.first)
			return false;
		else { // key.first == N2.key.first
			if (key.second < N2.key.second)
				return true;
			else if (key.second > N2.key.second)
				return false;
			else { // k1==k1 & k2==k2	->	to allow different nodes with the same key to be both in the queue
				//if (X != N2.X || Y != N2.Y)  -> CRASH -> comparator has to be strict
				if (X != N2.X)			
					return (X > N2.X);	//arbitrary order (not important)
				else if (Y != N2.Y)
					return (Y > N2.Y);
				else
					return false;
			}
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
		//std::cout << "[" << X << "," << Y << "] -> g=" << g << " , rhs=" << rhs << std::endl;						//without cost
		std::cout << "[" << X << "," << Y << "] -> g=" << g << " , rhs=" << rhs << " , c=" << +cost << std::endl;	//with cost
	}

	void print_Adjacents() {
		std::cout << "Adjacents to node ";
		print_Coord();
		std::cout << " : ";
		
		for (auto A_ptr : AdjacentsVect) {
			(*A_ptr).print_Coord();
			std::cout << "  ";
		}
		std::cout << std::endl;
	}
};



class dummyNode {
public:
	//char Name; //debug
	int X, Y;
	int cost;
	NodeTypes nodeType;

/////////////////////////   Constructors   //////////////////////////
	dummyNode() {}		// for pointers etc.

	dummyNode(int x, int y, int ec, NodeTypes flag) // for actual nodes
		: X(x), Y(y), cost(ec), nodeType(flag) {}
};