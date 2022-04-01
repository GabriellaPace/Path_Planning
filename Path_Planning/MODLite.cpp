#include <iostream>
#include <limits>		// for min/max
//#include <optional>   // -> might be useful for key & predecessor


// Node = state in Koeing
class Node {
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
	Entity(int x, int y, bool isStartNode, bool isGoalNode) {
		X = x;
		Y = y;

		isStart = isStartNode;
		isGoal = isGoalNode;

		key.first = -1;
		key.second = -1;
		predecessor = nullptr;

		g = std::numeric_limits<float>::infinity();
		rhs = std::numeric_limits<float>::infinity();
	}
};



int main() {
	std::cin.get();
}