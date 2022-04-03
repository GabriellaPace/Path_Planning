#pragma once
#include "Functions.h"

class Node {			// Node = state in Koeing
public:
	////// common to all istances: //////
	static float k_m;// = 0.0f;
	static Node* ptrToStart;// = nullptr;
	static std::list<Node*> allNodes;

	/////////////////////////////////////
	char Name; // only for debug

	int X;
	int Y;

	bool isStart;
	bool isGoal;

	float g;
	float rhs;
	//float cost;
	//float h;

	std::pair<float, float> key;

	void* predecessor;

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	Node(char name, int x, int y, bool isStartNode, bool isGoalNode) {  // for actual nodes
		Name = name;
		X = x;
		Y = y;

		isStart = isStartNode;
		isGoal = isGoalNode;

		if (isStartNode) {
			ptrToStart = this;
		}


		if (isGoal) {
			g = 0.0f;
			rhs = 0.0f;
		}
		else {
			g = std::numeric_limits<float>::infinity();
			rhs = std::numeric_limits<float>::infinity();
		}

		key.first = -1;
		key.second = -1;
		predecessor = nullptr;
		
		allNodes.push_front(this);
	}

	//~Node() {
	//	allNodes.remove(this);
	//}

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

	//bool operator < (const Node &N2) const {
	//	if (key.first < N2.key.first)
	//		return true;
	//	else if (key.first > N2.key.first)
	//		return false;
	//	else { // key.first == N2.key.first
	//		if (key.second < N2.key.second)
	//			return true;
	//		else // (key.second >= N2.key.second)
	//			return false;
	//	}
	//}
	//bool operator == (const Node &N2) const {
	//	if (key.first == N2.key.first  &&  key.second == N2.key.second)
	//		return true;
	//	else
	//		return false;
	//}

/////////////////////////////////////   Methods   //////////////////////////////////
	void calculateKey() {
		key.second = nonDom(g, rhs);
		key.first = key.second + heuristic() + k_m;
	}


	float heuristic() {
		static Node startNode = *ptrToStart;
		static int X_start = startNode.X;
		static int Y_start = startNode.Y;

		std::cout << "X and Y of start node: " << X_start << " , " << Y_start << std::endl;

		float h = (float)sqrt(pow((X - X_start), 2.0f) + pow((Y - Y_start), 2.0f));    //pow(base, power)
		std::cout << "heuristic of node n" << Name << " : " << h << std::endl << std::endl;
		return h;
	}


//////////////////////////////////   debug Methods   ////////////////////////////////
	//void print_NodeName() {
	//	std::cout << Name;
	//}

	void print_NodeKey() {
		std::cout << key.first << " , " << key.second << std::endl;
	}
};