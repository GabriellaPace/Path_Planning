#include "Functions.h"

std::vector<Wptr_toNode> solutionPaths;

int main() {
	updateMap();

// function PLAN():
	// function Initialize():
	calculateKey(ptrToGoal);
	queue.insert(*ptrToGoal);

	computeMOPaths();


	while (ptrToStart != ptrToGoal) {
		solutionPaths = generateMOPaths();
		if (solutionPaths.empty()) {
			std::cout << " => There are no avaliable paths for map {" << count-1 << "}, waiting for any edge cost to change.\n\n";
		}
		else {
			print_solution(solutionPaths);
		}

		//sleep(5);
		queue.clear();	//added by me -> right????????????????????

		updateMap();
	}
	std::cout << " => GOAL REACHED. Exiting.\n\n";
// end of function PLAN()

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	std::cout << " => END.\n\n";
	//std::cin.get();
}