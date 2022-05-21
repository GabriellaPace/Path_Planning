#include "Functions.h"

std::vector<Wptr_toNode> solutionPaths;

int main() {
	ReadMap_firstTime();
	for (auto N_ptr : NodesVect) {    // fill (and print) adjacents to each node 
		findAdjacents(N_ptr);
		(*N_ptr).print_Adjacents(); //debug
	} //(can't be done in constructor because not all nodes have been registered yet)

// function PLAN()
	// function Initialize()
	for (auto N_ptr : NodesVect) {
		if ( (*N_ptr).nodeType == goal ) {
			calculateKey(N_ptr);
			//queue.push(*N_ptr);
			queue.insert(*N_ptr);
		}
	}

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

		/*remove*/
		solutionPaths = generateMOPaths();
		if (solutionPaths.empty()) {
			std::cout << " => There are no avaliable paths - waiting for any edge cost to change.\n\n";
		}
		else {
			print_solution(solutionPaths);
		}
		/*remove*/

	//}
// end of function PLAN()

	// DELETE ALL objects -> shared_ptr are automatically deleted when out of scope (??) -> so no need to do it manually 
	std::cout << " => END.\n\n";
	std::cin.get();
}