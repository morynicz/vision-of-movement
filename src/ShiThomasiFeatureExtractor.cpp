/*
 * ShiThomasCornersExtractor.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "ShiThomasiFeatureExtractor.hpp"

#include "opencv2/video/tracking.hpp"

const double DEFAULT_QUALITY_LEVEL = 0.1;
const double DEFAULT_MIN_DISTANCE = 10;
const double DEFAULT_BLOCK_SIZE = 5;
const cv::Size DEFAULT_WIN_SIZE = cv::Size(5, 5);
const cv::Size DEFAULT_ZERO_ZONE_SIZE = cv::Size(-1, -1);
const cv::TermCriteria DEFAULT_TERM_CRITERIA = cv::TermCriteria(
		CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

ShiThomasiFeatureExtractor::ShiThomasiFeatureExtractor() :
		_qualityLevel(DEFAULT_QUALITY_LEVEL), _minDistance(
				DEFAULT_MIN_DISTANCE), _blockSize(DEFAULT_BLOCK_SIZE), _winSize(
				DEFAULT_WIN_SIZE), _zeroZone(DEFAULT_ZERO_ZONE_SIZE), _termCrit(
				DEFAULT_TERM_CRITERIA) {

}

ShiThomasiFeatureExtractor::ShiThomasiFeatureExtractor(
		const ShiThomasiFeatureExtractor &toCopy) :
		_qualityLevel(toCopy._qualityLevel), _minDistance(toCopy._minDistance), _blockSize(
				toCopy._blockSize), _winSize(toCopy._winSize), _zeroZone(
				toCopy._zeroZone), _termCrit(toCopy._termCrit) {

}

ShiThomasiFeatureExtractor::ShiThomasiFeatureExtractor(const double &qualityLevel,
		const double &minDistance, const int &blockSize,
		const cv::Size &winSize, const cv::Size &zeroZone,
		const cv::TermCriteria &termCrit) :
		_qualityLevel(qualityLevel), _minDistance(minDistance), _blockSize(
				blockSize), _winSize(winSize), _zeroZone(zeroZone), _termCrit(
				termCrit) {

}

FeatureExtractor *ShiThomasiFeatureExtractor::constructCopy() const {
	return new ShiThomasiFeatureExtractor(*this);
}

ShiThomasiFeatureExtractor::~ShiThomasiFeatureExtractor() {

}

std::vector<cv::Point2f> ShiThomasiFeatureExtractor::extractFeatures(
		const cv::Mat &input, const int& maxCorners) const {
	std::vector<cv::Point2f> result;

	cv::goodFeaturesToTrack(input, result, maxCorners, _qualityLevel,
			_minDistance, cv::Mat(), _blockSize, false);
	if (!result.empty()) {
		cv::cornerSubPix(input, result, _winSize, _zeroZone, _termCrit);
	}
	return result;
}


cv::FileStorage &operator<<(cv::FileStorage &fs,
        const ShiThomasiParameters &shiThomasi) {
    fs << "{" << "qualityLevel" << shiThomasi.qualityLevel
            << "minDistance" << shiThomasi.minDistance << "blockSize"
            << shiThomasi.blockSize << "winSize" << shiThomasi.winSize
            << "zeroZone" << shiThomasi.zeroZone << "termCriteria"
            << shiThomasi.termCrit << "}";
    return fs;
}

void operator>>(const cv::FileNode &node,
        ShiThomasiParameters &shiThomasi) {
    node["blockSize"] >> shiThomasi.blockSize;
    node["qualityLevel"] >> shiThomasi.qualityLevel;
    node["minDistance"] >> shiThomasi.minDistance;
    node["winSize"] >> shiThomasi.winSize;
    node["zeroZone"] >> shiThomasi.zeroZone;
    node["termCriteria"] >> shiThomasi.termCrit;
}
