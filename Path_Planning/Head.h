#pragma once

#include <iostream>
#pragma once
#include <limits>		// for min/max
//#include <optional>   // -> might be useful for key & predecessor
#include <queue>
//#include <functional>
//#include <vector>
#include <cmath>
//#include <list>
#include <vector>
#include <string> // maybe only for debug ?
#include <memory> //for: shared_from_this()
#include <valarray> //for: parents.values().sum()
#include <unordered_map>


float nonDom(float g, float rhs) {			// nonDom = min		// still considering single g and rhs		
	if (g <= rhs) {
		//std::cout << "	(nonDom = g)" << std::endl;
		return g;
	}
	else {
		//std::cout << "	(nonDom = rhs)" << std::endl << std::endl;
		return rhs;
	}
}