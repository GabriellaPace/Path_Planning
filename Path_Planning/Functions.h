#pragma once

#include <iostream>
#include <limits>		// for min/max
//#include <optional>   // -> might be useful for key & predecessor
#include <queue>
//#include <functional>
//#include <vector>
#include <cmath>
//#include <list>
#include <vector>
#include <string> // maybe only for debug ?


float nonDom(float g, float rhs) {			// still considering single g and rhs
	if (g <= rhs) {
		std::cout << " g " << std::endl;
		return g;
	}
	else {
		std::cout << " rhs " << std::endl;
		return rhs;
	}
}