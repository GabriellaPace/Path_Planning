#pragma once
#include <iostream>
#include <cmath>
#include <limits> // for min/max
//#include <memory> //for: shared_from_this()
#include <queue>
#include <deque>
//#include <list>
#include <vector>
#include <string> // maybe only for debug ?
//#include <valarray> //for: parents.values().sum()
#include "robin_hood.h" //faster than <unordered_map>

//#define DEBUG  //#ifdef DEBUG   #endif
//all_of  -  none_of  -  any_of  -  find_if


float nonDom_2(float g, float rhs) {		// nonDom = min			
	return std::min(g, rhs);		// still considering single g and rhs	
}


enum domin_res {
	fst_dominates = 0, snd_dominates = 1, areEqual = 2, nonDomination = 3
};

domin_res domination(float fst, float snd) {		//vectDomination()
	if (fst > snd) {
		return fst_dominates;
	}
	else if (fst == snd) {
		return areEqual;
	}
	else if (fst < snd){
		return snd_dominates;
	}
	//else {	//usesell until fst and snd are float
	//	return nonDomination;
	//}
}

/*
domin_res vectDomination(std::vector<float> g, std::vector<float> rhs) {		// vectorial g and rhs [scalabile!]
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
		return fst_dominates;
	}
	else if ((countL + countE) == g.size()) {	// <=
		return snd_dominates;
	}
	else if (countE == g.size()) {
		return areEqual;
	}
	else {
		return nonDomination;
	}
}
*/

domin_res multi_dom(float c, std::vector<uint8_t> vectC) {	//compare a cost and a vector of costs
	if (std::all_of(vectC.begin(), vectC.end(), [&c](const uint8_t& v) {return (domination(c, v) == fst_dominates); } )) {
		return fst_dominates;
	}
	else if (std::all_of(vectC.begin(), vectC.end(), [&c](const uint8_t& v) {return (domination(c, v) == areEqual); })) {
		return areEqual;
	}
	else if (std::all_of(vectC.begin(), vectC.end(), [&c](const uint8_t& v) {return (domination(c, v) == snd_dominates); })) {
		return snd_dominates;
	}
	else {
		return nonDomination;
	}
}
// ^ there should be a better way to do it!!