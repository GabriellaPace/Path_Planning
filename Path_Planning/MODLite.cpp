#include <iostream>
#include <limits>		// for min/max
//#include <optional>   // -> might be useful for key & predecessor
#include <queue>
//#include <functional>
//#include <vector>


class Node {			// Node = state in Koeing
public:
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

	Node(int x, int y, bool isStartNode, bool isGoalNode) {
		X = x;
		Y = y;

		isStart = isStartNode;
		isGoal = isGoalNode;

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

	bool operator == (const Node &N2) const {
		if (key.first == N2.key.first  &&  key.second == N2.key.second)
			return true;
		else
			return false;
	}

	//////////////////////////////////////////////////////////////////////////////////
	void print_Node() {
		std::cout << key.first << " , " << key.second << std::endl;
	}
};



template<typename T>
void print_queue(T& q) {
	Node tmp;
	while (!q.empty()) {
		tmp = q.top();
		tmp.print_Node();
		q.pop();
	}
}


int main() {
	//std::priority_queue<Node, std::vector<Node>, CompareKey >  queue;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node> >  queue;

	//////////////////////////////////
	Node n1(0, 0, true, false);
	Node n2(1, 0, false, false);
	Node n3(2, 0, false, true);

	n1.key.first = 0.0f;
	n1.key.second = 0.0f;
	queue.push(n1);

	n2.key.first = 1.0f;
	n2.key.second = 4.0f;
	queue.push(n2);

	n3.key.first = 2.0f;
	n3.key.second = 0.0f;
	queue.push(n3);
	//////////////////////////////////

	print_queue(queue);
	std::cin.get();
}