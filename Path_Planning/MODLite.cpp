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
			std::cout << " => There are no avaliable paths - waiting for any edge cost to change.\n\n";
		}
		else {
			/*REMOVE*/
			if (count == 2) {
				for (int i = 0; i < 3; ++i) {
					NodesVect[i]->print_g_rhs();
				}
				std::cout << std::endl;
				for (int i = 3; i < 6; ++i) {
					NodesVect[i]->print_g_rhs();
				}
			}
			/*REMOVE*/
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