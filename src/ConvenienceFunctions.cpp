/*
 * ConvenienceFunctions.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */

#include "ConvenienceFunctions.hpp"
#include <iostream>

bool horizontalPoint3Compare(cv::Point3f p1, cv::Point3f p2) {
	return p1.x < p2.x;
}

bool verticalPoint3Compare(cv::Point3f p1, cv::Point3f p2) {
	return p1.y < p2.y;
}

void printMatrix(const cv::Mat &arg, bool printValues) {
	std::cerr << "dims: " << arg.cols << ' ' << arg.rows << std::endl;
	std::cerr << "chans: " << arg.channels() << std::endl;
	int type = arg.depth();
	std::string depth;
	switch (type) {
	case CV_8U:
		depth = "CV_8U";
		break;
	case CV_8S:
		depth = "CV_8S";
		break;
	case CV_16U:
		depth = "CV_16U";
		break;
	case CV_16S:
		depth = "CV_16S";
		break;
	case CV_32S:
		depth = "CV_32S";
		break;
	case CV_32F:
		depth = "CV_32F";
		break;
	case CV_64F:
		depth = "CV_64F";
		break;
	}
	std::cerr << "depth: " << depth << std::endl;

	if (printValues) {
		for (int i = 0; i < arg.rows; ++i) {
			for (int j = 0; j < arg.cols; ++j) {
				switch (type) {
				case CV_8U:
					std::cerr << (int) arg.at<unsigned char>(i, j);
					break;
				case CV_8S:
					std::cerr << arg.at<char>(i, j);
					break;
				case CV_16U:
					std::cerr << arg.at<unsigned int>(i, j);
					break;
				case CV_16S:
					std::cerr << arg.at<int>(i, j);
					break;
				case CV_32S:
					std::cerr << arg.at<long int>(i, j);
					break;
				case CV_32F:
					std::cerr << arg.at<float>(i, j);
					break;
				case CV_64F:
					std::cerr << arg.at<double>(i, j);
					break;
				}
				std::cerr << " ";
			}
			std::cerr << std::endl;
		}
	}
}



