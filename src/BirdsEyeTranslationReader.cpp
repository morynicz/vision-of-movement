/*
 * BirdsEyeTranslationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "BirdsEyeTranslationReader.hpp"
#include "ImageEdgeFilter.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
bool horizontalPoint2Compare(cv::Point2f p1, cv::Point2f p2) {
	return p1.x < p2.x;
}

bool verticalPoint2Compare(cv::Point2f p1, cv::Point2f p2) {
	return p1.y < p2.y;
}

cv::Mat drawFeatureHistory(const cv::Mat &newImage,
		std::vector<std::list<cv::Point2f> > featureHistory);

//TODO: rememmber to mke this rotation radius ini valid
BirdsEyeTranslationReader::BirdsEyeTranslationReader(const cv::Mat &homography,
		const FeatureExtractor &extractor, const FeatureTracker &tracker,
		const unsigned int& maxFeatures,
		const std::list<FeatureFilter*>& filters, cv::Point2f rotationCentre,
		const cv::Size& imageSize, const double &margin) :
		_maxFeatures(maxFeatures), _rotationCenter(rotationCentre) {
	_homography = homography.clone();
	_tracker = tracker.constructCopy();
	_extractor = extractor.constructCopy();
	for (std::list<FeatureFilter*>::const_iterator it = filters.begin();
			it != filters.end(); ++it) {
		_filters.push_back((*it)->constructCopy());
	}
	_filters.push_front(new ImageEdgeFilter(homography, imageSize, margin));

}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
		const BirdsEyeTranslationReader &toCopy) :
		_homography(toCopy._homography), _translations(toCopy._translations), _maxFeatures(
				toCopy._maxFeatures), _rotationCenter(toCopy._rotationCenter) {
	_trackedFeatures = toCopy._trackedFeatures;
	_tracker = toCopy._tracker->constructCopy();
	_extractor = toCopy._extractor->constructCopy();
	for (std::list<FeatureFilter*>::const_iterator it = toCopy._filters.begin();
			it != toCopy._filters.end(); ++it) {
		_filters.push_back((*it)->constructCopy());
	}
}

BirdsEyeTranslationReader::~BirdsEyeTranslationReader() {
	if (NULL != _tracker) {
		delete _tracker;
	}
	if (NULL != _extractor) {
		delete _extractor;
	}
	for (std::list<FeatureFilter*>::const_iterator it = _filters.begin();
			it != _filters.end(); ++it) {
		delete *it;
	}
}

TranslationReader *BirdsEyeTranslationReader::constructCopy() const {
	return new BirdsEyeTranslationReader(*this);
}

cv::Point3f BirdsEyeTranslationReader::readTranslation(const cv::Mat &newFrame,
		const double& rotationAngle) {
	cv::Mat newTransformed;
	cv::Point3f result(0, 0, rotationAngle);
	cv::Mat rotationMatrix = cv::getRotationMatrix2D(_rotationCenter,
			-rotationAngle, 1);
	int vectorHalf = 0;
	cv::warpPerspective(newFrame, newTransformed, _homography, newFrame.size(),
			cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
	if (!_oldFrame.empty()) {
		_extractor->refillFeatures(_oldFrame, _trackedFeatures, _maxFeatures);
		_tracker->trackFeatures(_oldFrame, newTransformed, _trackedFeatures);
		if (!_trackedFeatures.empty()) {
//			cv::Mat pre = drawFeatureHistory(newTransformed, _trackedFeatures);
//			imshow("c1", pre);
//			imshow("ol", _oldFrame);
			for (std::list<FeatureFilter*>::iterator it = _filters.begin();
					it != _filters.end(); ++it) {
				_trackedFeatures = (*it)->filterFeatures(_trackedFeatures);
			}
//			cv::Mat post = drawFeatureHistory(newTransformed, _trackedFeatures);
//			imshow("c2", post);
//			cv::waitKey(0);
			std::vector<cv::Point2f> translations = computeTranslationVectors(
					rotationMatrix);
			vectorHalf = translations.size() / 2;
			std::nth_element(translations.begin(),
					translations.begin() + vectorHalf, translations.end(),
					horizontalPoint2Compare);
			result.x = translations[vectorHalf].x;
			std::nth_element(translations.begin(),
					translations.begin() + vectorHalf, translations.end(),
					verticalPoint2Compare);
			result.y = translations[vectorHalf].y;
		}
	}
	_oldFrame = newTransformed.clone();
	return result;
}

std::vector<cv::Point2f> BirdsEyeTranslationReader::computeTranslationVectors(
		const cv::Mat& rotationMatrix) {
	std::vector<cv::Point2f> result(_trackedFeatures.size());
	std::vector<cv::Point2f> oldFeatures(_trackedFeatures.size()), newFeatures(
			_trackedFeatures.size());
	std::vector<cv::Point2f> newTransformed(newFeatures.size());

	for (unsigned int i = 0; i < result.size(); ++i) { //Highly susceptible to bugs
		oldFeatures[i] = _trackedFeatures[i].front();
		newFeatures[i] = *(++_trackedFeatures[i].begin());
	}

	cv::transform(newFeatures, newTransformed, rotationMatrix);
	for (unsigned int i = 0; i < result.size(); ++i) {
		result[i] = newTransformed[i] - oldFeatures[i];
	}

	return result;
}

cv::Mat drawFeatureHistory(const cv::Mat &newImage,
		std::vector<std::list<cv::Point2f> > featureHistory) {
	int radius = 5;
	cv::Mat result = newImage.clone();
	for (int i = 0; i < featureHistory.size(); ++i) {
		cv::circle(result, featureHistory[i].front(), radius,
				cv::Scalar(100, 0, 100), -1, 8, 0);
		std::list<cv::Point2f>::iterator itb = featureHistory[i].begin();
		std::list<cv::Point2f>::iterator ite = itb;
		if (!featureHistory.size() < 2) {
			for (ite++; ite != featureHistory[i].end(); ++ite, ++itb) {
				cv::line(result, *ite, *itb, CV_RGB(100,0,0), 1, CV_AA);
			}
		}
	}
	return result;
}
