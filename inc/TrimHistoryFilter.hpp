/*
 * TrimHistoryFilter.hpp
 *
 *  Created on: Mar 30, 2013
 *      Author: link
 */

#ifndef TRIMHISTORYFILTER_HPP_
#define TRIMHISTORYFILTER_HPP_

#include <FeatureFilter.hpp>

class TrimHistoryFilter: public FeatureFilter {
	unsigned _maxLength;
public:
	TrimHistoryFilter(const unsigned int &maxLength);
	TrimHistoryFilter(const TrimHistoryFilter &toCopy);
	std::vector<std::list<cv::Point2f> > filterFeatures(
			const std::vector<std::list<cv::Point2f> > &features);
	FeatureFilter* constructCopy()const;
	virtual ~TrimHistoryFilter();
};

#endif /* TRIMHISTORYFILTER_HPP_ */
