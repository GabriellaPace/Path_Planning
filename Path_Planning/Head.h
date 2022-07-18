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
#include "../External Libraries/tsl/robin_map.h" //faster than <map>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <time.h> //clock - only for debug

/*-------------------------------------------------------------------------------------------------------*/

int obj = 2;	//quantity of objectives we are considering
using flt_vect = std::vector<float>;


enum domin_res {
	fst_completely_dominates = 0, 	fst_dominates = 1, 
	snd_completely_dominates = 2,	snd_dominates = 3, 
	areEqual = 4, 
	nonDomination = 5
};	//I could swap the arguments in the functions and avoid "snd_dominates", but this is more readable -> is it a problem?


/*-------------------------------------------------------------------------------------------------------*/

domin_res single_domination(float fst, float snd) {
	if (fst < snd) {						//domination = smaller one
		return fst_completely_dominates; //fst_dominates;
	}
	else if (fst == snd) {
		return areEqual;
	}
	else {
		return snd_completely_dominates; //snd_dominates;
	}
}

//template <typename FI>	//Float-Integer
//domin_res domination(const std::vector<FI> fst, const std::vector<FI> snd) {
domin_res domination(flt_vect fst, flt_vect snd) {
	//fst and snd should have same size by definition [usually fst=g, snd=rhs]
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

	if (count_fst == fst.size()) {						// <
		return fst_completely_dominates;
	}
	else if (count_snd == fst.size()) {					// >
		return snd_completely_dominates;
	}
	else if (count_eq == fst.size()) {					// ==
		return areEqual;
	}
	else if ((count_fst + count_eq) == fst.size()) {	// <=
		std::cout << " [!!!] fst_dominates\n";
		return fst_dominates;
	}
	else if ((count_snd + count_eq) == fst.size()) {	// >=
		std::cout << " [!!!] snd_dominates\n";
		return snd_dominates;
	}
	else {
		return nonDomination;
		std::cout << " [!!!] nonDomination\n";
	}
}

/*-------------------------------------------------------------------------------------------------------*/

flt_vect nonDom(flt_vect g, flt_vect rhs) {
	if (domination(g, rhs) != snd_dominates  &&  domination(g, rhs) != snd_completely_dominates)	// = g is not dominated (= can be considered the min)
		return g;
	else
		return rhs;
	// if areEqual or nonDomination, return g (ok??) -> not specified in paper
}