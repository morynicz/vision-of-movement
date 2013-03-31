/*
 * TrimHistoryFilter.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: link
 */

#include "TrimHistoryFilter.hpp"

TrimHistoryFilter::TrimHistoryFilter(const unsigned int &maxLength) :
		_maxLength(maxLength) {

}

TrimHistoryFilter::~TrimHistoryFilter() {
}

TrimHistoryFilter::TrimHistoryFilter(const TrimHistoryFilter &toCopy) :
		_maxLength(toCopy._maxLength) {
}

FeatureFilter* TrimHistoryFilter::constructCopy() const {
	return new TrimHistoryFilter(*this);
}

std::vector<std::list<cv::Point2f> > TrimHistoryFilter::filterFeatures(
		const std::vector<std::list<cv::Point2f> > &features) {
	std::vector<std::list<cv::Point2f> > result(features);
	for (unsigned int i = 0; i < features.size(); ++i) {
			if (result[i].size() > _maxLength) {
				result[i].resize(_maxLength);
			}
		}
	return result;
}
