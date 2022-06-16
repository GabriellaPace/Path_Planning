#include "Functions.h"

std::vector<Wptr_toNode> solutionPaths;

int main() {
	/*
	Node n('m', 1, 0, 10, any);
	n.key.first = 30;
	n.key.second = 20;
	Node nS('M', 0, 0, 10, start);
	nS.key.first = 30;
	nS.key.second = 20;
	if (n < nS)
		std::cout << "OK (n < nS)\n";
	else
		std::cout << "WRONG (n > nS)\n";
	*/

	updateMap();

// function PLAN():
	// function Initialize():
	calculateKey(ptrToGoal);
	queue.insert(*ptrToGoal);

	computeMOPaths();


	while (ptrToStart != ptrToGoal) {
		solutionPaths = generateMOPaths();
		if (solutionPaths.empty()) {
			std::cout << " => There are no avaliable paths for map {" << map_count-1 << "}, waiting for any edge cost to change.\n\n";
		}
		else {
			print_solution(solutionPaths);
		}

		//sleep(5);
		queue.clear();	//added by me -> maybe uselees (hopefully not wrong)

		updateMap();
	}
	std::cout << " => GOAL REACHED. Exiting.\n\n";
// end of function PLAN()

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	//std::cin.get();
}