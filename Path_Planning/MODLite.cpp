#include "Functions.h"


int main() {
	startTime = clock();	//debug
	startTime_whole = clock();	//debug

	updateMap();

	if (successful_read) {
		// function PLAN():
			// function Initialize():
			calculateKey(ptrToGoal);
			queue.insert(*ptrToGoal);

		computeMOPaths();
	}

	while (ptrToStart != ptrToGoal) {
	//while (map_count < 100) {
		if (successful_read) {
			solutionPaths = generateMOPaths();
			if (solutionPaths.empty()) {
				std::cout << " => There are no avaliable paths for map {" << map_count << "}, waiting for any edge cost to change.\n\n";
			}
			else {
				save_solution_img(solutionPaths);
				std::cout << "\n => Solution found.\n";
				std::cout << "    Total cycle time: " << double(clock() - startTime_whole)/(double)CLOCKS_PER_SEC << " s.\n\n";
			}
		}
		//sleep(5);
		queue.clear();	//added by me -> maybe useless (hopefully not wrong)	��

		++map_count;
		startTime_whole = clock();	//debug
		updateMap();
	}
	std::cout << " => GOAL REACHED. Exiting.\n\n";
// end of function PLAN()

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
}