/*
 * SmoothFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "SmoothFilter.hpp"
#include <cmath>
#include <algorithm>

SmoothFilter::SmoothFilter(const double& maxDeviation,
		const double& deviantThreshold) :
		_maxDeviation(maxDeviation), _deviantThreshold(deviantThreshold) {
	// TODO Auto-generated constructor stub

}

SmoothFilter::~SmoothFilter() {
	// TODO Auto-generated destructor stub
}

std::vector<std::list<cv::Point2f> > SmoothFilter::filterFeatures(
		const std::vector<std::list<cv::Point2f> >& features) {

	std::vector<bool> toKill(features.size());
	std::vector<std::list<cv::Point2f> > result(features);
	int counter = 0;
	for (int i = 0; i < toKill.size(); ++i) {
		if (3 > features[i].size()) {
			toKill[i] = false;
		} else {
			std::list<cv::Point2f>::const_iterator it = features[i].begin();
			cv::Point2f first = *it;
			cv::Point2f second = *(++it);
			cv::Point2f third = *(++it);
			cv::Vec2f v1 = third - second;
			cv::Vec2f v2 = second - first;
			if (_maxDeviation
					< acos(
							(v1[0] * v2[0] + v1[1] * v2[1])
									/ (cv::norm(v1) + cv::norm(v2)))) {
				toKill[i] = true;
				++counter;
			} else {
				toKill[i] = false;
			}
		}
	}

	if (_deviantThreshold >= counter) {
		for (int j = 0; j < toKill.size(); ++j) {
			if (toKill[j]) { //WARNING: bug may occure here
				result.erase(result.begin() + j--);
			}
		}
	}
	return result;
}

