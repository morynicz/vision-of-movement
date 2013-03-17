/*
 * FeatureExtractor.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "FeatureExtractor.hpp"


bool comparePoints(cv::Point2f p1, cv::Point2f p2) {
	if (p1.x != p2.x) {
		return p1.x < p2.x;
	} else {
		return p1.y < p2.y;
	}
	return false;
}

bool compareListPoints(std::list<cv::Point2f> l1, std::list<cv::Point2f> l2) {
	cv::Point2f p1 = l1.front();
	cv::Point2f p2 = l2.front();

	if (p1.x != p2.x) {
		return p1.x < p2.x;
	} else if (p1.y != p2.y) {
		return p1.y < p2.y;
	} else if (l1.size() != l2.size()) {
		return l1.size() > l2.size();
	}
	return false;
}

bool listPointsEqual(std::list<cv::Point2f> l1, std::list<cv::Point2f> l2) {
	return l1.front() == l2.front();
}


FeatureExtractor::FeatureExtractor() {
	// TODO Auto-generated constructor stub

}

FeatureExtractor::~FeatureExtractor() {
	// TODO Auto-generated destructor stub
}

void FeatureExtractor::refillFeatures(const cv::Mat& oldFrame,
		std::vector<std::list<cv::Point2f> >& features,
		const unsigned int& maxFeatures) {
	std::vector<std::list<cv::Point2f> > result(features);

	if (features.size() > maxFeatures) {
		features.resize(maxFeatures);
	} else if (features.size() < maxFeatures) {
		std::vector<cv::Point2f> oldFeatures;
		for (unsigned int i = 0; i < features.size(); ++i) {
			oldFeatures.push_back(features[i].front());
		}
		std::vector<std::list<cv::Point2f> > additionalFeatureList;
		std::vector<cv::Point2f> additionalFeatures = extractFeatures(oldFrame,
				maxFeatures - oldFeatures.size());
		additionalFeatureList.resize(additionalFeatures.size());
		for (unsigned int i = 0; i < additionalFeatures.size(); ++i) {
			additionalFeatureList[i].push_front(additionalFeatures[i]);
		}
		features.insert(features.end(),
				additionalFeatureList.begin(), additionalFeatureList.end());
		std::sort(features.begin(), features.end(),
				compareListPoints);
		features.erase(
				std::unique(features.begin(), features.end(),
						listPointsEqual));
	}
}
