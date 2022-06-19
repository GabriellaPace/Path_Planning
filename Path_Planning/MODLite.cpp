#include "Functions.h"


std::vector<Sptr_toNode> solutionPaths;

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

	if (successful_read) {
		// function PLAN():
			// function Initialize():
		calculateKey(ptrToGoal);
		queue.insert(*ptrToGoal);

		computeMOPaths();
	}

	//while (ptrToStart != ptrToGoal) {
	while (map_count < 2) {
		if (successful_read) {
			solutionPaths = generateMOPaths();
			if (solutionPaths.empty()) {
				std::cout << " => There are no avaliable paths for map {" << map_count << "}, waiting for any edge cost to change.\n\n";
			}
			else {
				print_solution(solutionPaths);
				save_solution_img(solutionPaths);
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