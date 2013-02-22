/*
 * ShiThomasCornersExtractor.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "ShiThomasFeatureExtractor.hpp"

#include "opencv2/video/tracking.hpp"

const double DEFAULT_QUALITY_LEVEL = 0.1;
const double DEFAULT_MIN_DISTANCE = 10;
const double DEFAULT_BLOCK_SIZE = 5;
const cv::Size DEFAULT_WIN_SIZE = cv::Size(5, 5);
const cv::Size DEFAULT_ZERO_ZONE_SIZE = cv::Size(-1, -1);
const cv::TermCriteria DEFAULT_TERM_CRITERIA = cv::TermCriteria(
		CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

ShiThomasFeatureExtractor::ShiThomasFeatureExtractor() :
		_qualityLevel(DEFAULT_QUALITY_LEVEL), _minDistance(
				DEFAULT_MIN_DISTANCE), _blockSize(DEFAULT_BLOCK_SIZE), _winSize(
				DEFAULT_WIN_SIZE), _zeroZone(DEFAULT_ZERO_ZONE_SIZE), _termCrit(
				DEFAULT_TERM_CRITERIA) {

}

ShiThomasFeatureExtractor::ShiThomasFeatureExtractor(
		const ShiThomasFeatureExtractor &toCopy) :
		_qualityLevel(toCopy._qualityLevel), _minDistance(toCopy._minDistance), _blockSize(
				toCopy._blockSize), _winSize(toCopy._winSize), _zeroZone(
				toCopy._zeroZone), _termCrit(toCopy._termCrit) {

}

ShiThomasFeatureExtractor::ShiThomasFeatureExtractor(const double &qualityLevel,
		const double &minDistance, const int &blockSize,
		const cv::Size &winSize, const cv::Size &zeroZone,
		const cv::TermCriteria &termCrit) :
		_qualityLevel(qualityLevel), _minDistance(minDistance), _blockSize(
				blockSize), _winSize(winSize), _zeroZone(zeroZone), _termCrit(
				termCrit) {

}

FeatureExtractor *ShiThomasFeatureExtractor::constructCopy() const {
	return new ShiThomasFeatureExtractor(*this);
}

ShiThomasFeatureExtractor::~ShiThomasFeatureExtractor() {

}

std::vector<cv::Point2f> ShiThomasFeatureExtractor::extractFeatures(
		const cv::Mat &input, const int& maxCorners) const {
	std::vector<cv::Point2f> result;

	cv::goodFeaturesToTrack(input, result, maxCorners, _qualityLevel,
			_minDistance, cv::Mat(), _blockSize, false);
	if (!result.empty()) {
		cv::cornerSubPix(input, result, _winSize, _zeroZone, _termCrit);
	}
	return result;
}
