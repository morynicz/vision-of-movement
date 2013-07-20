/*
 * LucasCandaePyramidTracker.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "LucasKanadePyramidTracker.hpp"
#include "opencv2/video/tracking.hpp"

const cv::TermCriteria DEFAULT_TERM_CRITERIA = cv::TermCriteria(
		CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 40, 0.001);
const cv::Size DEFAULT_WINDOW_SIZE = cv::Size(21, 21);
const unsigned int DEFAULT_MAX_LEVEL = 3;
const unsigned int DEFAULT_FLAGS = 0;
const double DEFAULT_MIN_EIGENVALUE_THRESHOLD = 1e-4;
const double DEFAULT_MAX_ERROR_THRESHOLD = 500;

LucasKanadePyramidTracker::LucasKanadePyramidTracker() :
		_windowSize(DEFAULT_WINDOW_SIZE), _maxLevel(DEFAULT_MAX_LEVEL), _flags(
				DEFAULT_FLAGS), _minEigenvalueThreshold(
				DEFAULT_MIN_EIGENVALUE_THRESHOLD), _maxErrorThreshold(
				DEFAULT_MAX_ERROR_THRESHOLD), _termCrit(DEFAULT_TERM_CRITERIA) {
}

LucasKanadePyramidTracker::~LucasKanadePyramidTracker() {
	// TODO Auto-generated destructor stub
}

LucasKanadePyramidTracker::LucasKanadePyramidTracker(
		const LucasKanadePyramidTracker &toCopy) :
		_windowSize(toCopy._windowSize), _maxLevel(toCopy._maxLevel), _flags(
				toCopy._flags), _minEigenvalueThreshold(
				toCopy._minEigenvalueThreshold), _maxErrorThreshold(
				toCopy._maxErrorThreshold), _termCrit(toCopy._termCrit) {

}

LucasKanadePyramidTracker::LucasKanadePyramidTracker(const cv::Size &windowSize,
		const unsigned int &maxLevel, const int &flags,
		const cv::TermCriteria &terminationCriteria,
		const double &minEigenvalueThreshold, const double &maxErrorValue) :
		_windowSize(windowSize), _maxLevel(maxLevel), _flags(flags), _minEigenvalueThreshold(
				minEigenvalueThreshold), _maxErrorThreshold(maxErrorValue), _termCrit(
				terminationCriteria) {

}

FeatureTracker *LucasKanadePyramidTracker::constructCopy() const {
	return new LucasKanadePyramidTracker(*this);
}

void LucasKanadePyramidTracker::trackFeatures(const cv::Mat &oldInput,
		const cv::Mat &newInput,
		std::vector<std::list<cv::Point2f> > &trackedFeatures) {
	std::vector<uchar> status;
	std::vector<float> err; //thou shall not use double instead of float!!
	std::vector<cv::Point2f> oldFeatures(trackedFeatures.size());
	std::vector<cv::Point2f> newFeatures(trackedFeatures.size());
	for (unsigned int i = 0; i < trackedFeatures.size(); ++i) {
		oldFeatures[i] = trackedFeatures[i].front();
	}
	cv::calcOpticalFlowPyrLK(oldInput, newInput, oldFeatures, newFeatures,
			status, err, _windowSize, _maxLevel, _termCrit, _flags,
			_minEigenvalueThreshold);

	for (unsigned i = 0; i < newFeatures.size(); ++i) {
		if (!status[i] || (err[i] > _maxErrorThreshold)) {
			newFeatures.erase(newFeatures.begin() + i);
			//oldFeatures.erase(oldFeatures.begin() + i);
			status.erase(status.begin() + i);
			err.erase(err.begin() + i);
			trackedFeatures.erase(trackedFeatures.begin() + i);
			--i;
		} else {
			trackedFeatures[i].push_front(newFeatures[i]);
		}
	}
}

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const LucasKanadeParameters &lucasKanade) {
    fs << "{" << "maxLevel" << lucasKanade.maxLevel << "flags"
            << lucasKanade.flags << "winSize" << lucasKanade.winSize
            << "minEigenvalueThresh"
            << lucasKanade.minEigenvalueThresh << "maxErrorValue"
            << lucasKanade.maxErrorValue << "termCrit"
            << lucasKanade.termCrit << "}";
    return fs;
}

void operator>>(const cv::FileNode &node,
        LucasKanadeParameters &lucasKanade) {
    node["maxLevel"] >> lucasKanade.maxLevel;
    node["flags"] >> lucasKanade.flags;
    node["winSize"] >> lucasKanade.winSize;
    node["minEigenvalueThresh"] >> lucasKanade.minEigenvalueThresh;
    node["maxErrorValue"] >> lucasKanade.maxErrorValue;
    node["termCrit"] >> lucasKanade.termCrit;
}
