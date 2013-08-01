/*
 * SmoothFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "SmoothFilter.hpp"
#include <cmath>
#include <algorithm>

SmoothFilter::SmoothFilter(const double& maxDeviation,
        const double& deviantThreshold, const double &minLength,
        const double &maxLength) :
        _maxDeviation(maxDeviation), _deviantThreshold(
                deviantThreshold), _minLength(minLength), _maxLength(
                maxLength) {

}

SmoothFilter::SmoothFilter(const SmoothFilter &toCopy) :
        _maxDeviation(toCopy._maxDeviation), _deviantThreshold(
                toCopy._deviantThreshold), _minLength(
                toCopy._minLength), _maxLength(toCopy._maxLength) {

}

SmoothFilter::~SmoothFilter() {
    // TODO Auto-generated destructor stub
}

std::vector<std::list<cv::Point2f> > SmoothFilter::filterFeatures(
        const std::vector<std::list<cv::Point2f> >& features) {

    std::vector<bool> toKill(features.size());
    std::vector<std::list<cv::Point2f> > result(features);
    int counter = 0;
    for (unsigned int i = 0; i < toKill.size(); ++i) {
        if (8 > features[i].size()) {
            toKill[i] = false;
        } else {
            std::list<cv::Point2f>::const_iterator it =
                    features[i].begin();
            cv::Point2f zeroth = *it;
            cv::Point2f first = *(++it);
            ++it;
            cv::Point2f third = *(++it);
            ++it;
            ++it;
            ++it;
            cv::Point2f seventh = *(++it);

            cv::Vec2f v10 = first - zeroth;
            cv::Vec2f v31 = third - first;
            cv::Vec2f v73 = seventh - third;

            double cos1031 = v10.dot(v31)
                    / (cv::norm(v10) * cv::norm(v31)) * 180 / CV_PI;
            double cos3173 = v31.dot(v73)
                    / (cv::norm(v31) * cv::norm(v73)) * 180 / CV_PI;
            double cos1073 = v10.dot(v73)
                    / (cv::norm(v10) * cv::norm(v73)) * 180 / CV_PI;

            if (_maxDeviation < acos(abs(cos1031))
                    || _maxDeviation < acos(abs(cos1073))
                    || _maxDeviation < acos(abs(cos3173))) {
                toKill[i] = true;
                ++counter;
            } else {
                toKill[i] = false;
            }
        }
    }

    if (_deviantThreshold >= counter / (double) features.size()) {
        for (unsigned int j = 0; j < toKill.size(); ++j) {
            if (toKill[j]) {
                result.erase(result.begin() + j);
                toKill.erase(toKill.begin() + j);
                --j;
            }
        }
    } else {
    }
    return result;
}

FeatureFilter *SmoothFilter::constructCopy() const {
    SmoothFilter *copy = new SmoothFilter(*this);
    return copy;
}

