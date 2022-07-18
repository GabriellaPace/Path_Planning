#pragma once
#include "ClassNode.h"

std::vector<Sptr_toNode> solutionPaths;
std::string img_path = "C:/Dev/Path_Planning/Maps/";
int map_count = 0;	// to change map in different iterations
bool successful_read = false;

#define BLUR_MAP
//#define MANUAL	//to manually insert the XYinates of start and goal each time


#ifdef BLUR_MAP	/*focused map*/
void ReadMap() {
	newMap.clear();

	cv::Mat img_mat, img_focus;
	std::pair<int, int> start_coord, coord;
	std::pair<int, int> goal_coord = { 420, 40 }; //{420, 480};	//FIXED

	//float x0=0;	//original
	//float x1=255;	//original
	float y0=10;//new
	float y1=11;//new

	img_mat = cv::imread((img_path + "image_blurred.bmp").c_str(), cv::IMREAD_GRAYSCALE);
	if (!img_mat.empty()) {
		if (map_count == 0) {
			//int pixelValue;
			flt_vect pixelValue;
			for (int x = 0; x < img_mat.rows; ++x) {
				for (int y = 0; y < img_mat.cols; ++y) {
					coord = { x, y };
					//pixelValue.push_back( (float)(img_mat.at<uchar>(x, y) * tune) ); // gray
					/*linear interpolation    yp = y0 + ((y1-y0)/(x1-x0)) * (xp - x0)  */		//x0=0, x1=255
					pixelValue.push_back( y0 + ((y1 - y0) / 255) * (float)(img_mat.at<uchar>(x, y)) );
					//pixelValue.push_back( y0 + ((y1 - y0) / 255) * (float)(img_mat.at<uchar>(x, y)) );	//second cost
 
					newMap[coord] = std::make_shared<dummyNode>(coord, pixelValue);
					pixelValue.clear();
				}
			}

			start_coord = { 10, 10 };
		}
		else {
			if (solutionPaths.size() >= 50)
				start_coord = solutionPaths[50]->XY;		//walk 10 nodes each time
			else
				start_coord = solutionPaths.back()->XY;		//pick last item
		}

		img_focus = cv::imread((img_path + "image_focus.bmp").c_str(), cv::IMREAD_GRAYSCALE);
		if (!img_focus.empty()) {
			//int pixelValue;
			flt_vect pixelValue;
			for (int x = (start_coord.first - 50); x < (start_coord.first + 50); ++x) {				//reading in a 20x20 square around the rover position
				for (int y = start_coord.second - 50; y < start_coord.second + 50; ++y) {
					if (x > 0 && x < img_focus.rows   &&   y>0 && y < img_focus.cols) {		//check feasibility
						coord = { x, y };
						//pixelValue.push_back( (float)(img_focus.at<uchar>(x, y) * tune) ); // gray
						/*linear interpolation*/
						pixelValue.push_back (y0 + ((y1 - y0) / 255) * (float)(img_focus.at<uchar>(x, y)) );
						//pixelValue.push_back( y0 + ((y1 - y0) / 255) * (float)(img_focus.at<uchar>(x, y)) );	//second cost

						newMap[coord] = std::make_shared<dummyNode>(coord, pixelValue);

						img_mat.at<uchar>(x, y) = img_focus.at<uchar>(x, y);	//needed just for graphics
						pixelValue.clear();

					}
				}
			}
			successful_read = true;
		}
		else {
			std::cout << "    error in focus for Map : {" << map_count << "}\n";
		}



		if (newMap.find(start_coord) != newMap.end())
			newMap[start_coord]->nodeType = start;
		else
			std::cout << "OUT OF RANGE!\n" << img_mat.rows << "  x  " << img_mat.rows << std::endl;


		if (newMap.find(goal_coord) != newMap.end())
			newMap[goal_coord]->nodeType = goal;		//FIXED GOAL

		cv::imwrite((img_path + std::to_string(map_count) + "_image_SOL.bmp").c_str(), img_mat);	//then it will be over-written (again just graphics)
	}
	std::cout << "*********************************************\n => MAP UPDATE N.{" << map_count << "}\n";
}
#else	/*100 - 500x500 maps*/
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
#endif // BLUR_MAP


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