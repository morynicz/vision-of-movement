/*
 * VisualOdometer.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: Michał Orynicz
 */

#include "VisualOdometer.hpp"
#include "TangentRotationReader.hpp"
#include "ShiThomasFeatureExtractor.hpp"
#include "LucasCandaePyramidTracker.hpp"
#include "BirdsEyeTranslationReader.hpp"

const unsigned int DEFAULT_MAX_FEATURES = 500;


VisualOdometer::~VisualOdometer() {
	if (NULL != _translationReader) {
		delete _translationReader;
	}
	if (NULL != _rotationReader) {
		delete _rotationReader;
	}

}

VisualOdometer::VisualOdometer(const RotationReader &rotationReader,
		const TranslationReader &translationReader,
	 const int &horizonHeight,
		const int &deadZoneWidth, const int &featuresNumber) :
		_featuresNumber(featuresNumber), _horizonHeight(horizonHeight), _deadZoneWidth(
				deadZoneWidth) {
	_translationReader = translationReader.constructCopy();
	_rotationReader = rotationReader.constructCopy();

}

cv::Point3f VisualOdometer::calculateDisplacement(const cv::Mat &newFrame) {
	cv::Point3f result(0, 0, 0);
	  cv::Rect lowerRoi(cv::Point2f(0, _horizonHeight + _deadZoneWidth),
	            cv::Size(newFrame.cols, newFrame.rows - _horizonHeight - _deadZoneWidth));
	    cv::Rect upperRoi(cv::Point2f(0, 0),
	            cv::Size(newFrame.cols, _horizonHeight - _deadZoneWidth));
	double alpha = _rotationReader->readRotation(newFrame(upperRoi));
	result = _translationReader->readTranslation(newFrame(lowerRoi), alpha);

	return result;
}

std::vector<std::list<cv::Point2f> > VisualOdometer::getRotationFeatures()const{
    return _rotationReader->getTrackedFeatures();
}

std::vector<std::list<cv::Point2f> > VisualOdometer::getTranslationFeatures() const{
    return _translationReader->getTrackedFeatures();
}
