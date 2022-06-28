#pragma once
#include <iostream>
#include <cmath>
#include <limits> //for min/max
#include <queue>
#include <deque>
#include <set>	//used instead of queue(?)
#include <vector>
#include <algorithm> // std::sort
#include <string> //maybe only for debug ?
#include "../External Libraries/robin_hood.h" //faster than <unordered_map>
#include "../External Libraries/tsl/robin_map.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>

#include <time.h> //clock

//#define DEBUG  //#ifdef DEBUG   #endif
/* all_of  -  none_of  -  any_of  -  find_if */

enum domin_res {
	//fst_completely_dominates = 0, 
	fst_dominates = 1, 
	//snd_completely_dominates = 2,
	snd_dominates = 3, 
	areEqual = 4, nonDomination = 5
};	//I could swap the arguments in the functions and avoid "snd_dominates", but this is more readable -> is it a problem?


domin_res domination(float fst, float snd) {
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

// vectorial version:
/*
domin_res domination(std::vector<float> fst, std::vector<float> snd) {		// vectorial g and rhs [scalabile!]
	//fst and snd should have same size by definition (usually fst=g, snd=rhs)
	int count_fst = 0;	//dominations of First parameter
	int count_snd = 0;	//dominations of Second parameter
	int count_eq = 0;	//equalities
	
	for (int i = 0; i < fst.size(); ++i) {			// domination = is min
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

	if (count_fst  == fst.size()) {						// <
		return fst_completely_dominates;
	}
	else if ((count_fst + count_eq) == fst.size()) {	// <=
		return fst_dominates;
	}
	else if (count_snd == fst.size()) {					// >
		return snd_completely_dominates;
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


float nonDom(float g, float rhs) {		// nonDominated (=dominant) = min			
	return std::min(g, rhs);		// still considering single g and rhs	
}

// vectorial version:
/*
std::vector<float> nonDom(std::vector<float> g, std::vector<float> rhs) {		// nonDominated (=dominant) = min
	if (domination(g, rhs) != snd_completely_dominates && domination(g, rhs) != snd_completely_dominates)	// g is not dominated
		return g;
	else
		return rhs;
	// if areEqual or nonDomination, return g (ok??) -> not specified in paper
}
*/