#pragma once
#include <iostream>
#include <cmath>
#include <limits>		// for min/max
#include <memory> //for: shared_from_this()
#include <queue>
#include <deque>
//#include <list>
#include <vector>
#include <string> // maybe only for debug ?
//#include <valarray> //for: parents.values().sum()
//#include <numeric> //for: std::accumulate()
#include "robin_hood.h" //faster than <unordered_map>

//#define DEBUG  //#ifdef DEBUG   #endif


float nonDom_2(float g, float rhs) {		// nonDom = min			
	return std::min(g, rhs);		// still considering single g and rhs	
}

bool nonDom_b(float a, float b) {
	if (a <= b) {
		return true;
	}
	else {
		return false;
	}
}


/*
//  is greater than : completely dominates
//	is smaller than : is completely dominated by
//	is equal to	    : is multiobjetively euqual
//					  nondomination

enum comparResult {
	g_isGreater = 0, rhs_isGreater = 1, g_rhs_areEqual = 2, g_rhs_nonDom = 3
};

comparResult vectComparison(std::vector<float> g, std::vector<float> rhs) {		// vectorial g and rhs [scalabile!]
	int countG = 0;
	int countL = 0;
	int countE = 0;

	for (int i = 0; i < g.size(); ++i) {
		if (g[i] > rhs[i]) {
			++countG;
		}
		else if (g[i] < rhs[i]) {
			++countL;
		}
		else {
			++countE;
		}
	}

	if ( (countG + countE) == g.size()) {		// >=
		return g_isGreater;
	}
	else if ((countL + countE) == g.size()) {	// <=
		return rhs_isGreater;
	}
	else if (countE == g.size()) {
		return g_rhs_areEqual;
	}
	else {
		return g_rhs_nonDom;
	}
}
*/