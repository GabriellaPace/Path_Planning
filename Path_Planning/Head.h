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
//#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>
#include <time.h> //clock - only for debug


//#define MULTI_OBJ
int obj = 1;	//quantity of objectives we are considering
using flt_vect = std::vector<float>;

enum domin_res {
	fst_completely_dominates = 0, 	fst_dominates = 1, 
	snd_completely_dominates = 2,	snd_dominates = 3, 
	areEqual = 4, 
	nonDomination = 5
};	//I could swap the arguments in the functions and avoid "snd_dominates", but this is more readable -> is it a problem?



//template<typename T>
//struct is_vector {
//	static constexpr bool value = false;
//};
//
////template<template<typename...> class C, typename U>
////struct is_vector<C<U>> {
////	static constexpr bool value = std::is_same<C<U>, std::vector<U>>::value;
////};
//
//		/* domination() */
//template <typename FI>	//Float-Integer -> SINGLE-OBJ version
//std::enable_if_t<!is_vector<FI>::value, FI> domination(const FI fst, const FI snd) {
//	if (fst < snd) {						//domination = smaller one
//		return fst_dominates;
//	}
//	else if (fst == snd) {
//		return areEqual;
//	}
//	else if (fst > snd) {
//		return snd_dominates;
//	}
//	else {	//useless until fst and snd are float
//		return nonDomination;
//	}
//}
//
//template <typename FIV>	//Float-Integer Vectors -> MULTI-OBJ version
//std::enable_if_t<is_vector<FIV>::value, FIV> domination(const FIV fst, const FIV snd) {
//	//fst and snd should have same size by definition [usually fst=g, snd=rhs]
//	int count_fst = 0;	//dominations of First parameter
//	int count_snd = 0;	//dominations of Second parameter
//	int count_eq  = 0;	//equalities
//
//	for (int i = 0; i < fst.size(); ++i) {			// domination = is min
//		if (fst[i] < snd[i]) {
//			++count_fst;
//		}
//		else if (fst[i] > snd[i]) {
//			++count_snd;
//		}
//		else {	//fst[i] == snd[i]
//			++count_eq;
//		}
//	}
//
//	if (count_fst == fst.size()) {						// <
//		return fst_completely_dominates;
//	}
//	else if ((count_fst + count_eq) == fst.size()) {	// <=
//		return fst_dominates;
//	}
//	else if (count_snd == fst.size()) {					// >
//		return snd_completely_dominates;
//	}
//	else if ((count_snd + count_eq) == fst.size()) {	// >=
//		return snd_dominates;
//	}
//	else if (count_eq == fst.size()) {					// ==
//		return areEqual;
//	}
//	else {
//		return nonDomination;
//	}
//}
//
///*-------------------------------------------------------------------------------------------------------*/
//
//		/* nonDom() */
//template <typename FI>	//Float-Integer -> SINGLE-OBJ version
//std::enable_if_t<!is_vector<FI>::value, FI> nonDom(const FI g, const FI rhs) {
//	return std::min(g, rhs);
//}
//
//
//template <typename FIV>	//Float-Integer Vectors -> MULTI-OBJ version
//std::enable_if_t<is_vector<FIV>::value, FIV> nonDom(const FIV g, const FIV rhs) {
//	if (domination(g, rhs) != snd_completely_dominates && domination(g, rhs) != snd_completely_dominates)	// g is not dominated
//		return g;
//	else
//		return rhs;
//	// if areEqual or nonDomination, return g (ok??) -> not specified in paper
//}



domin_res single_domination(float fst, float snd) {
	if (fst < snd) {						//domination = smaller one
		return fst_dominates;
	}
	else if (fst == snd) {
		return areEqual;
	}
	else if (fst > snd) {
		return snd_dominates;
	}
	else {	//useless until fst and snd are float
		return nonDomination;
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
		return fst_dominates;
	}
	else if ((count_snd + count_eq) == fst.size()) {	// >=
		return snd_dominates;
	}
	else {
		return nonDomination;
	}
}

/*-------------------------------------------------------------------------------------------------------*/

//template <typename FI>	//Float-Integer
//std::vector<FI> nonDom(const std::vector<FI> g, const std::vector<FI> rhs) {
flt_vect nonDom(flt_vect g, flt_vect rhs) {
	if (domination(g, rhs) != snd_dominates  &&  domination(g, rhs) != snd_completely_dominates)	// = g is not dominated
		return g;
	else
		return rhs;
	// if areEqual or nonDomination, return g (ok??) -> not specified in paper
}