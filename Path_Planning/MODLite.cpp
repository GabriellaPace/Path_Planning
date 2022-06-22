#include "Functions.h"


std::vector<Sptr_toNode> solutionPaths;


int main() {
	startTime = clock();

	updateMap();

	if (successful_read) {
		// function PLAN():
			// function Initialize():
		calculateKey(ptrToGoal);
		queue.insert(*ptrToGoal);

		computeMOPaths();
	}

	//while (ptrToStart != ptrToGoal) {
	while (map_count < 20) {
		if (successful_read) {
			solutionPaths = generateMOPaths();
			if (solutionPaths.empty()) {
				std::cout << " => There are no avaliable paths for map {" << map_count << "}, waiting for any edge cost to change.\n\n";
			}
			else {
				//print_solution(solutionPaths);
				save_solution_img(solutionPaths);
				std::cout << "\n => Solution found.\n\n";
			}
		}
		//sleep(5);
		queue.clear();	//added by me -> maybe uselees (hopefully not wrong)

		++map_count;
		updateMap();
	}
	std::cout << " => GOAL REACHED. Exiting.\n\n";
// end of function PLAN()

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	//std::cin.get();
}