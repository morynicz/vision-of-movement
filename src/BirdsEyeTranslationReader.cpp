/*
 * BirdsEyeTranslationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "BirdsEyeTranslationReader.hpp"
#include "ImageEdgeFilter.hpp"
#include <opencv2/imgproc/imgproc.hpp>


#include "DrawingFunctions.hpp"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

bool horizontalPoint2Compare(cv::Point2f p1, cv::Point2f p2) {
    return p1.x < p2.x;
}

bool verticalPoint2Compare(cv::Point2f p1, cv::Point2f p2) {
    return p1.y < p2.y;
}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
        const cv::Mat &homography, const FeatureExtractor &extractor,
        const FeatureTracker &tracker,
        const unsigned int& maxFeatures,
        const std::list<FeatureFilter*>& filters,
        cv::Point2f rotationCentre, const cv::Size& imageSize,
        const double &margin, const cv::Size &viewSize) :
        TranslationReader(tracker, extractor, filters, maxFeatures), _rotationCenter(
                rotationCentre), _viewSize(viewSize), _homography(
                homography.clone()) {
    _filters.push_front(
            new ImageEdgeFilter(homography, imageSize, margin));

}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
        const BirdsEyeTranslationReader &toCopy) :
        TranslationReader(toCopy), _translations(
                toCopy._translations), _rotationCenter(
                toCopy._rotationCenter), _viewSize(toCopy._viewSize), _homography(
                toCopy._homography) {
}

BirdsEyeTranslationReader::~BirdsEyeTranslationReader() {
}

TranslationReader *BirdsEyeTranslationReader::constructCopy() const {
    return new BirdsEyeTranslationReader(*this);
}

cv::Point3f BirdsEyeTranslationReader::readTranslation(
        const cv::Mat &newFrame, const double& rotationAngle) {
    cv::Mat newTransformed;
    cv::Point3f result(0, 0, rotationAngle);
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(_rotationCenter,
            rotationAngle, 1);
    int vectorHalf = 0;
    cv::warpPerspective(newFrame, newTransformed, _homography,
            newFrame.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
    // imshow("c2", newTransformed);
    //cv::waitKey(1);
    if (!_oldFrame.empty()) {
        _extractor->refillFeatures(_oldFrame, _trackedFeatures,
                _maxFeatures);
        _tracker->trackFeatures(_oldFrame, newTransformed,
                _trackedFeatures);
        if (!_trackedFeatures.empty()) {
//			cv::Mat pre = drawFeatureHistory(newTransformed, _trackedFeatures);
//			imshow("c1", pre);
//			imshow("ol", _oldFrame);
            for (std::list<FeatureFilter*>::iterator it =
                    _filters.begin(); it != _filters.end(); ++it) {
                _trackedFeatures = (*it)->filterFeatures(
                        _trackedFeatures);
            }
//			cv::Mat post = drawFeatureHistory(newTransformed, _trackedFeatures);

            std::vector<cv::Point2f> translations =
                    computeTranslationVectors(rotationMatrix);
            vectorHalf = translations.size() / 2;
            std::nth_element(translations.begin(),
                    translations.begin() + vectorHalf,
                    translations.end(), horizontalPoint2Compare);
            result.x = translations[vectorHalf].x;
            std::nth_element(translations.begin(),
                    translations.begin() + vectorHalf,
                    translations.end(), verticalPoint2Compare);
            result.y = translations[vectorHalf].y;
        }
    }
    _oldFrame = newTransformed.clone();
    return result;
}

std::vector<cv::Point2f> BirdsEyeTranslationReader::computeTranslationVectors(
        const cv::Mat& rotationMatrix) {
    std::vector<cv::Point2f> result(_trackedFeatures.size());
    std::vector<cv::Point2f> oldFeatures(_trackedFeatures.size()),
            newFeatures(_trackedFeatures.size());
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

