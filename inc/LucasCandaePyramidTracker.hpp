/*
 * LucasCandaePyramidTracker.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef LUCASCANDAEPYRAMIDTRACKER_HPP_
#define LUCASCANDAEPYRAMIDTRACKER_HPP_

#include <FeatureTracker.hpp>

class LucasCandaePyramidTracker: public FeatureTracker {
	cv::Size _windowSize;
	unsigned int _maxLevel;
	int _flags;
	double _minEigenvalueThreshold;
	double _maxErrorThreshold;
	cv::TermCriteria _termCrit;
public:
	LucasCandaePyramidTracker();
	LucasCandaePyramidTracker(const LucasCandaePyramidTracker &toCopy);
	LucasCandaePyramidTracker(const cv::Size &windowSize,
			const unsigned int &maxLevel, const int &flags,
			const cv::TermCriteria &terminationCriteria,
			const double &minEigenvalueThreshold, const double &maxErrorValue);
	virtual FeatureTracker *constructCopy() const;
	virtual void trackFeatures(const cv::Mat &oldInput, const cv::Mat &newInput,
			std::vector<std::list<cv::Point2f> > &trackedFeatures);
	virtual ~LucasCandaePyramidTracker();
};

#endif /* LUCASCANDAEPYRAMIDTRACKER_HPP_ */
