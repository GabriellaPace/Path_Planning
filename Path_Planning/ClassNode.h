#pragma once
#include "Head.h"


class Node;		 //forward declaration
class dummyNode; //forward declaration
using Sptr_toNode  = std::shared_ptr<Node>;
using Sptr_toDummy = std::shared_ptr<dummyNode>;

/*--------------------------------------------------  robin_maps  --------------------------------------------------*/
// tsl::hopscotch_map if you don�t want to use too much memory,  tsl::robin_map if speed is what mainly matters  ->  tsl::robin_map	
struct hash_pair {
	template <class T1, class T2>
	size_t operator()(const std::pair<T1, T2>& p) const {
		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);

		if (hash1 != hash2) {
			return hash1 ^ hash2;
		}

		return hash1;	// If hash1 == hash2, their XOR is zero.
	}
};
bool operator < (const std::pair<int, int>&  pair1, const std::pair<int, int>&  pair2) {	//overwrite operator < for std::pair<int,int> -> used for coord
	if (pair1.first < pair2.first)
		return true;
	else if (pair1.first > pair2.first)
		return false;
	else { // X == dc2.X
		if (pair1.second < pair2.second)
			return true;
		else	// Y >= N2.Y
			return false;
	}
};

tsl::robin_map<std::pair<int, int>, Sptr_toNode,  hash_pair> allNodes;	// map of shared pointers to Nodes (declaration and definition)
tsl::robin_map<std::pair<int, int>, Sptr_toDummy, hash_pair> newMap;	// map of shared pointers to dummyNodes

/*------------------------------------------------------------------------------------------------------------------*/
enum NodeTypes {
	start = 0, any = 1, goal = 2
};

/*-----------------------------------------------  Node & dummyNode  -----------------------------------------------*/

class Node {		// Node = state in Koeing
public:
	std::pair<int, int> XY;	// X = XY.first,  Y = XY.second

	/* SINGLE-OBJ */
	//int cost;   //read from height map (from which we will derive edge costs)
	//float g, rhs;	//g = cost function,	rhs = one step lookahead value of g
	//robin_hood::unordered_map < Sptr_toNode, int > parents;    //key: ptr to node, value: cumulative cost

	/* MULTI-OBJ  */
	std::vector<float> cost, g, rhs;	//in SINGLE-OBJ are vectors with a single element
	robin_hood::unordered_map < Sptr_toNode, std::vector<float> > parents;    //key: ptr to node, value: cumulative cost(s)

	NodeTypes nodeType;
	std::pair<float, float> key;
	std::vector<Sptr_toNode> AdjacentsVect;	//all nodes adjacent to current one (const??)

	bool expanded = false;

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	//template <typename IV>
	Node(std::pair<int,int> coord, flt_vect ec, NodeTypes nodeType)  // for actual nodes
		: XY(coord), cost(ec), nodeType(nodeType) {

		//obj = number of objectives to optimize
		for (int i=0; i<obj; ++i)		g.push_back( std::numeric_limits<float>::infinity() );

		if (nodeType == goal) {
			for (int i = 0; i < obj; ++i)	rhs.push_back(0.0f);	}
		else {
			for (int i = 0; i < obj; ++i)	rhs.push_back(std::numeric_limits<float>::infinity());		}


		//g = std::numeric_limits<float>::infinity();
		//if (nodeType == goal)
		//	rhs = 0.0f;
		//else
		//	rhs = std::numeric_limits<float>::infinity();

		key.first  = -1;
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
				if (XY.first != N2.XY.first)
					return (XY.first > N2.XY.first);	//arbitrary order (not important)
				else if (XY.second != N2.XY.second)
					return (XY.second > N2.XY.second);
				else
					return false;
			}
		}
	}

//////////////////////////////////   debug Methods   ////////////////////////////////
	void print_Coord() {
		std::cout << "[" << XY.first << "," << XY.second << "]";
	}

	void print_NodeKey() {
		std::cout << "( "<< key.first <<" , "<< key.second <<" ) -> [" << XY.first << "," << XY.second << "]" << std::endl;
	}

	//void print_g_rhs() {
	//	std::cout << "[" << XY.first << "," << XY.second << "] -> g=" << g << " , rhs=" << rhs << " , c=" << +cost << std::endl;	//with cost
	//}

	//void print_Adjacents() {
	//	std::cout << "Adjacents to node ";
	//	print_Coord();
	//	std::cout << " : ";
	//	
	//	for (auto A_ptr : AdjacentsVect) {
	//		(*A_ptr).print_Coord();
	//		std::cout << "  ";
	//	}
	//	std::cout << std::endl;
	//}
};




class dummyNode {
public:
	std::pair<int, int> XY;
	flt_vect cost;
	NodeTypes nodeType = any;	//default: generic node

/////////////////////////   Constructors   //////////////////////////
	dummyNode() {}		// for pointers etc.
	dummyNode(std::pair<int,int> XY, flt_vect ec) // for actual nodes
		: XY(XY), cost(ec) {}
};