#pragma once
#include "ClassNode.h"

std::vector<Sptr_toNode> solutionPaths;

std::string img_path = "C:/Dev/Path_Planning/Maps/";
int map_count = 0;	// to change map in different iterations
bool successful_read = false;
//#define MANUAL	//to manually insert the XYinates of start and goal each time


/*focused map*/
//void ReadMap() {
//	newMap.clear();
//
//	cv::Mat img_mat, img_focus;
//	std::pair<int, int> coord;
//
//
//	if (map_count == 0) {
//		img_mat = cv::imread((img_path + "image_blurred.bmp").c_str(), cv::IMREAD_GRAYSCALE);
//
//		if (!img_mat.empty()) {
//			int pixelValue;
//			for (int x = 0; x < img_mat.rows; ++x) {
//				for (int y = 0; y < img_mat.cols; ++y) {
//					coord = { x, y };
//					pixelValue = static_cast<int>(img_mat.at<uchar>(x, y)); // gray
//
//					newMap[coord] = std::make_shared<dummyNode>(coord, pixelValue);
//				}
//			}
//		}
//
//
//		coord = { 10, 10 };
//		if (newMap.find(coord) != newMap.end()) {
//			newMap[coord]->nodeType = start;
//		}
//		else {
//			std::cout << "OUT OF RANGE!\n" << img_mat.rows << "  x  " << img_mat.cols << std::endl;
//		}
//
//		coord = { 420, 480 };							//FIXED GOAL
//		if (newMap.find(coord) != newMap.end()) {
//			newMap[coord]->nodeType = goal;
//		}
//		else {
//			std::cout << "OUT OF RANGE!\n" << img_mat.rows << "  x  " << img_mat.cols << std::endl;
//		}
//
//	}
//	else {
//		std::pair<int, int> new_start;
//		if(solutionPaths.size() > 9)
//			new_start = solutionPaths[10]->XY;		//walk 10 nodes each time
//		else
//			new_start = solutionPaths.back()->XY;	//pick last item
//
//		if (newMap.find(new_start) != newMap.end())
//			newMap[new_start]->nodeType = start;
//		else
//			std::cout << "OUT OF RANGE!\n" << img_mat.rows << "  x  " << img_mat.rows << std::endl;
//
//
//		img_focus = cv::imread((img_path + "image_focus.bmp").c_str(), cv::IMREAD_GRAYSCALE);
//		if (!img_focus.empty()) {
//			int pixelValue;
//			for (int     x = new_start.first-10;  x < new_start.first+10;  ++x) {				//reading in a 20x20 square around the rover position
//				for (int y = new_start.second-10; y < new_start.second+10; ++y) {
//					if (x > 0 && x < img_focus.rows   &&   y>0 && y < img_focus.cols) {		//check feasibility
//						coord = {x, y};
//						pixelValue = static_cast<int>(img_mat.at<uchar>(x, y)); // gray
//
//						img_mat.at<uchar>(x, y) = pixelValue;
//						newMap[coord] = std::make_shared<dummyNode>(coord, pixelValue);
//					}
//				}
//			}
//		}
//	}
//
//	
//	cv::imwrite((img_path + std::to_string(map_count) + "_image_SOL.bmp").c_str(), img_mat);	//then it will be over-written
//
//	std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << map_count << "}\n";
//	successful_read = true;
//}



/*100 - 500x500 maps*/
void ReadMap() {
	newMap.clear();

	cv::Mat img_mat = cv::imread((img_path + std::to_string(map_count) + "_gradient.bmp").c_str(), cv::IMREAD_GRAYSCALE);
																		  // cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
	if (!img_mat.empty()) {
		//int* pixelPtr = (int*)img_mat.data;
		//int cn = img_mat.channels();
		std::pair<int, int> coord;
		int pixelValue;	//cv::Scalar_<int> bgrPixel;
		for (int x = 0; x < img_mat.rows; ++x) {
			for (int y = 0; y < img_mat.cols; ++y) {
				//bgrPixel.val[0] = pixelPtr[x*img_mat.cols*cn + y*cn + 0]; // B
				//bgrPixel.val[1] = pixelPtr[x*img_mat.cols*cn + y*cn + 1]; // G
				//bgrPixel.val[2] = pixelPtr[x*img_mat.cols*cn + y*cn + 2]; // R
				coord = {x, y};
				pixelValue = static_cast<int>(img_mat.at<uchar>(x, y)); // gray

				if (pixelValue == 0)
					pixelValue = 1;

				//newMap[coord] =  std::make_shared<dummyNode>(coord, pixelValue);
				newMap[coord] = std::make_shared<dummyNode>(coord, static_cast<int>(pixelValue * 255));
			}
		}


		// setting start and goal nodes:
		Sptr_toDummy dummy;

		#ifdef MANUAL
			int x_in, y_in;
			std::cout << "Insert coordinates of START node: x y\n";
			std::cin >> x_in; std::cin >> y_in;
			while (x_in < 0 || x_in > img_mat.rows || y_in < 0 || y_in > img_mat.rows) {
				std::cout << "Start node coordinates are out of range, please insert new ones (Xmax = " << img_mat.rows 
																						 << ", Ymax = " << img_mat.rows << "):\n";
				std::cin >> x_in; std::cin >> y_in;
			}
			//...
		#endif // MANUAL

		#ifndef MANUAL
			coord = {10, 10};
			if (newMap.find(coord) != newMap.end()) {		//if (newMap[coord] != NULL)	DOESN'T WORK!!!
				newMap[coord]->nodeType = start;
			}
			else {
				std::cout << "OUT OF RANGE!\n" << img_mat.rows << "  x  " << img_mat.cols << std::endl;
			}
			coord = {420, 480};
			if (newMap.find(coord) != newMap.end()) {
				newMap[coord]->nodeType = goal;
			}
			else {
				std::cout << "OUT OF RANGE!\n" << img_mat.rows << "  x  " << img_mat.cols << std::endl;
			}
		#endif // !MANUAL

		std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << map_count << "}\n";
		successful_read = true;
	}
	else {
		std::cout << "\n ^ Image for map {" << map_count << "} not read correctly.\n\n";
		successful_read = false;
	}
}

//Node insertion by hand (no .bmp) (ReadMap())
/*
void ReadMap() {
	newMap.clear();

	if (map_count == 0) {
		newMap.push_back(std::make_shared<dummyNode>('0', 0, 0, 10, start));
		newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 20, any));
		newMap.push_back(std::make_shared<dummyNode>('2', 2, 0, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('3', 0, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, goal));
		++map_count;
	}
	else if (map_count == 1) {
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 60, any));
		
		//newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 5, any));
		//newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 2, goal));
		//reducing only the cost of (1,0) doesn't have the same effect, because the cost keeps being the max beetween two nodes
		++map_count;
	}
	else if (map_count == 2) {
		newMap.push_back(std::make_shared<dummyNode>('0', 0, 0, 10, any));	//ASSUMPTION: this fix always happen (so we don't heve 2 "start")
		//newMap.push_back(std::make_shared<dummyNode>('1', 1, 0, 20, start));
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 60, start));
		newMap.push_back(std::make_shared<dummyNode>('6', 3, 0, 10, any));
		++map_count;
	}
	else if (map_count == 3) {	//goal change
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, any));
		newMap.push_back(std::make_shared<dummyNode>('2', 2, 0, 50, any));
		newMap.push_back(std::make_shared<dummyNode>('6', 3, 0, 10, goal));
		++map_count;
	}
	else if (map_count == 4) {
		newMap.push_back(std::make_shared<dummyNode>('4', 1, 1, 60, any));
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, start)); //and goal  ->  ptrToStart == ptrToGoal  ->  ARRIVED
		newMap.push_back(std::make_shared<dummyNode>('5', 2, 1, 10, goal));	 //and goal  ->  ptrToStart == ptrToGoal  ->  ARRIVED

		++map_count;
	}
	else {
		return;
	}

	std::cout << "*********************************************\n => RECEIVED NEW MAP: {" << map_count-1 << "}\n";
}
*/