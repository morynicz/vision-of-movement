/*
 * ShiThomasCornersExtractor.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef SHITHOMASFEATUREEXTRACTOR_HPP_
#define SHITHOMASFEATUREEXTRACTOR_HPP_

#include "FeatureExtractor.hpp"

class ShiThomasFeatureExtractor: public FeatureExtractor {
	double _qualityLevel;
	double _minDistance;
	int _blockSize;
	cv::Size _winSize;
	cv::Size _zeroZone;
	cv::TermCriteria _termCrit;
public:
	ShiThomasFeatureExtractor();
	ShiThomasFeatureExtractor(const double &qualityLevel,
			const double &minDistance, const int &blockSize,
			const cv::Size &winSize, const cv::Size &zeroZone,
			const cv::TermCriteria &termCrit);
	ShiThomasFeatureExtractor(const ShiThomasFeatureExtractor &toCopy);
	virtual FeatureExtractor *constructCopy() const;
	virtual std::vector<cv::Point2f> extractFeatures(const cv::Mat &input,
			const int & maxCorners)const;
	virtual ~ShiThomasFeatureExtractor();
};

#endif /* SHITHOMASFEATUREEXTRACTOR_HPP_ */
