#pragma once
#include "Functions.h"

enum NodeTypes {
	start = 0, any = 1, goal = 2
};


class Node {			// Node = state in Koeing
public:
	////// common to all istances: ////////////////////////
	static std::vector<Node> NodesList; //def. in ReadMap.h
	static float k_m;		//def. in MODLite.h
	static int X_start;		//		"
	static int Y_start;		//		"
	/*---------------------------------------------------*/
	char Name; //debug

	int X;
	int Y;

	NodeTypes nodeType;

	float g;
	float rhs;
	//float cost;

	std::pair<float, float> key;

	void* predecessor;

////////////////////////////////////   Constructors   /////////////////////////////////
	Node() {}		// for pointers etc.

	Node(char name, int x, int y, NodeTypes flag) {  // for actual nodes
		Name = name;
		X = x;
		Y = y;

		nodeType = flag;
		//isStart = isStartNode;
		//isGoal = isGoalNode;

		if (nodeType == start) {
			//ptrToStart = this;
			X_start = X;
			Y_start = Y;
		}

		g = std::numeric_limits<float>::infinity();

		if (nodeType == goal) {
			rhs = 0.0f;
			//queue.push(*this);	//can't do this here
		}
		else {
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


	int heuristic() {		// shortest aereal path (ignoring the grid)
		std::cout << "X and Y of start node: " << X_start << " , " << Y_start << std::endl;
		int h = (int)((sqrt(pow((X - X_start), 2.0f) + pow((Y - Y_start), 2.0f)))*10);  //pow(base, power)
		std::cout << "heuristic of node [" << X << "," << Y << "] : " << h << std::endl << std::endl;
		return h;
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
	void print_NodeKey() {
		std::cout << "( "<< key.first <<" , "<< key.second <<" ) -> [" << X << "," << Y << "]" << std::endl;
	}
};