/*
 * ShiThomasCornersExtractor.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef SHITHOMASFEATUREEXTRACTOR_HPP_
#define SHITHOMASFEATUREEXTRACTOR_HPP_

#include "FeatureExtractor.hpp"
#include "CvTypesIo.hpp"

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

class ShiThomasiParameters {
    public:
        double qualityLevel;
        double minDistance;
        int blockSize;
        cv::Size winSize;
        cv::Size zeroZone;
        cv::TermCriteria termCrit;

};

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const ShiThomasiParameters &shiThomasi);

void operator>>(const cv::FileNode &node,
        ShiThomasiParameters &shiThomasi);

#endif /* SHITHOMASFEATUREEXTRACTOR_HPP_ */
