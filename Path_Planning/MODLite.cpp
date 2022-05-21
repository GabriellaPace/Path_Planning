#include "Functions.h"

std::vector<Wptr_toNode> solutionPaths;

int main() {
	updateMap();

// function PLAN():
	// function Initialize():
	calculateKey(ptrToGoal);
	queue.insert(*ptrToGoal);

	//printAll_g_rhs();   print_queue();  //debug
	computeMOPaths();

	//while (start != goal) {
		solutionPaths = generateMOPaths();
		if (solutionPaths.empty()) {
			std::cout << " => There are no avaliable paths - waiting for any edge cost to change.\n\n";
		}
		else {
			print_solution(solutionPaths);
		}

		//sleep(5);
		updateMap();
	//}
// end of function PLAN()

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	std::cout << " => END.\n\n";
	std::cin.get();
}