#pragma once
#include "Functions.h"

class Node {			// Node = state in Koeing
public:
	////// common to all istances: ////////////////////////
	static std::vector<Node> NodesList; //def. in ReadMap.h
	static float k_m;		//def. in MODLite.h
	static int X_start;		//		"
	static int Y_start;		//		"
	/*---------------------------------------------------*/
	char Name; // only for debug

	int X;
	int Y;

	bool isStart;
	bool isGoal;

	float g;
	float rhs;
	//float cost;

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
			//ptrToStart = this;
			X_start = X;
			Y_start = Y;
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

		NodesList.push_back(*this);
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


	float heuristic() {
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
		std::cout << Name << " : " << key.first << " , " << key.second << std::endl;
	}
};