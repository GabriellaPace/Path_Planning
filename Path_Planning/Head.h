#pragma once
#include <iostream>
#include <cmath>
#include <limits> //for min/max
#include <queue>
#include <deque>
#include <set>	//used instead of queue(?)
//#include <list>
#include <vector>
#include <string> //maybe only for debug ?
#include "robin_hood.h" //faster than <unordered_map>


//#define DEBUG  //#ifdef DEBUG   #endif
/* all_of  -  none_of  -  any_of  -  find_if */


float nonDom_2(float g, float rhs) {		// nonDominated (=dominant) = min			
	return std::min(g, rhs);		// still considering single g and rhs	
}


enum domin_res {
	fst_dominates = 0, snd_dominates = 1, areEqual = 2, nonDomination = 3
};

domin_res domination(float fst, float snd) {		//vectDomination()
	if (fst < snd) {						//domination = smaller one
		return fst_dominates;
	}
	else if (fst == snd) {
		return areEqual;
	}
	else if (fst > snd){
		return snd_dominates;
	}
	else {	//useless until fst and snd are float
		return nonDomination;
	}
}

/*
domin_res vectDomination(std::vector<float> fst, std::vector<float> snd) {		// vectorial g and rhs [scalabile!]
	//fst and snd should have same size by definition (usually fst=g, snd=rhs)
	int count_fst = 0;	//dominations of First parameter
	int count_snd = 0;	//dominations of Second parameter
	int count_eq = 0;	//equalities

	// domination = is min
	for (int i = 0; i < fst.size(); ++i) {
		if (fst[i] < snd[i]) {
			++count_fst;
		}
		else if (fst[i] > snd[i]) {
			++count_snd;
		}
		else {	//fst[i] == snd[i]
			++count_eq;
		}
	}

	if ( (count_fst + count_eq) == fst.size()) {		// <=
		return fst_dominates;
	}
	else if ((count_snd + count_eq) == fst.size()) {	// >=
		return snd_dominates;
	}
	else if (count_eq == fst.size()) {					// ==
		return areEqual;
	}
	else {
		return nonDomination;
	}
}
*/

domin_res multi_dom(float c, std::vector<uint8_t> vectC) {	//compare a cost and a vector of costs
	if (std::all_of(vectC.begin(), vectC.end(), [&c](const uint8_t& v) {return (domination(c, v) == fst_dominates); } )) {		// < (not <=)  -> ok?
		return fst_dominates;	//fst_completely_cominates
	}
	else if (std::all_of(vectC.begin(), vectC.end(), [&c](const uint8_t& v) {return (domination(c, v) == areEqual); })) {		// ==
		return areEqual;
	}
	else if (std::all_of(vectC.begin(), vectC.end(), [&c](const uint8_t& v) {return (domination(c, v) == snd_dominates); })) {	// > (not >=)  -> ok?
		return snd_dominates;	//snd_completely_cominates
	}
	else {
		return nonDomination;
	}
}	// ^ or is it better with counters? 
