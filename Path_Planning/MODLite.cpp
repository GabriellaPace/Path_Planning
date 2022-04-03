#include <iostream>
#include <limits>		// for min/max
//#include <optional>   // -> might be useful for key & predecessor
#include <queue>
//#include <functional>
//#include <vector>
#include <cmath>

#include "Functions.h"

float k_m = 0.0f;
void* ptrToStart = nullptr;


class Node {			// Node = state in Koeing
public:
	char Name; // only for debug

	int X;
	int Y;

	bool isStart;
	bool isGoal;

	float g;
	float rhs;
	//float cost;
	float h;

	std::pair<float, float> key;

	void* predecessor;

	// constructor:
	Node() {}

	Node(char name, int x, int y, bool isStartNode, bool isGoalNode) {
		Name = name;
		X = x;
		Y = y;

		isStart = isStartNode;
		isGoal = isGoalNode;

		if (isStartNode) {
			ptrToStart = (void*)this;
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
	}


	// to sort the queue  ////////////////////////////////////////////////////////////
	//struct CompareKey {
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
		//return result; //return key.second > N2.key.second;
	}
	//};

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

	//////////////////////////////////////////////////////////////////////////////////
	void calculateKey(){
		key.second = nonDom(g, rhs);
		key.first = key.second + heuristic() + k_m;
	}


	float heuristic() {
		static Node startNode = *((Node*)ptrToStart);
		static int X_start = startNode.X;
		static int Y_start = startNode.Y;

		std::cout << "X and Y of start node: " << X_start << " , " << Y_start << std::endl;
			
		float h = (float)sqrt( pow((X - X_start), 2.0f) + pow((Y - Y_start), 2.0f));    //pow(base, power)
		std::cout << "heuristic of node n" << Name << " : " << h << std::endl << std::endl;
		return h;
	}
	

	//////////////////////////////////////////////////////////////////////////////////
	//void print_NodeName() {
	//	std::cout << Name;
	//}

	void print_NodeKey() {
		std::cout << key.first << " , " << key.second << std::endl;
	}
};


template<typename T>
void print_queue(T& q) {
	Node tmp;
	while (!q.empty()) {
		tmp = q.top();
		tmp.print_NodeKey();
		q.pop();
	}
	std::cout << std::endl;
}


int main() {
	//Node* ptrToStart = nullptr;
	///////////////////////////////////////////////////////

	//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node> >  queue;

	///////////////////////////////////////////////////////
	Node n1('1', 0, 0, true, false);
	Node n2('2', 1, 0, false, false);
	Node n3('3', 2, 0, false, true);

	n1.key.first = 0.0f;
	n1.key.second = 0.0f;
	queue.push(n1);

	n2.key.first = 1.0f;
	n2.key.second = 4.0f;
	queue.push(n2);

	n3.key.first = 2.0f;
	n3.key.second = 0.0f;
	//n3.key.first = 1.0f;   //works anyway
	//n3.key.second = 4.0f;
	queue.push(n3);

	print_queue(queue);

	///////////////////////////////////////////////////////
	n1.heuristic();
	n2.heuristic();
	n3.heuristic();

	///////////////////////////////////////////////////////

	std::cin.get();
}