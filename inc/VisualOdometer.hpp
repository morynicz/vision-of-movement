/*
 * VisualOdometer.h
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef VISUALODOMETER_H_
#define VISUALODOMETER_H_

#include "TranslationReader.hpp"
#include "RotationReader.hpp"
#include "FeatureFilter.hpp"
#include "SmoothFilter.hpp"
#include "ImageEdgeFilter.hpp"
#include "FeatureTracker.hpp"
#include "FeatureExtractor.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <list>
#include <cmath>

class VisualOdometer {
	RotationReader *_rotationReader;
	TranslationReader *_translationReader;
	std::list<FeatureFilter*> _filters;
	std::vector<std::list<cv::Point2f> > trackedFeatures;
	int _featuresNumber;
	int _horizonHeight;
	int _deadZoneWidth;
	cv::RNG _rng;
public:
	VisualOdometer(const RotationReader &rotationReader,
			const TranslationReader &translationReader,
			const std::list<FeatureFilter*> filters,
			const int &horizonHeight, const int &deadZoneWidth,
			const int &featuresNumber);
	cv::Point3f calculateDisplacement(const cv::Mat &newFrame);
	virtual ~VisualOdometer();
};

#endif /* VISUALODOMETER_H_ */
