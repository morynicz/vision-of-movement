/*
 * SmoothFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "SmoothFilter.hpp"
#include <cmath>
#include <algorithm>

#include<iostream>
using namespace std;

SmoothFilter::SmoothFilter(const double& maxDeviation,
        const double& deviantThreshold, const double &minLength) :
        _maxDeviation(maxDeviation), _deviantThreshold(deviantThreshold), _minLength(
                minLength) {
    // TODO Auto-generated constructor stub

}

SmoothFilter::SmoothFilter(const SmoothFilter &toCopy) :
        _maxDeviation(toCopy._maxDeviation), _deviantThreshold(
                toCopy._deviantThreshold), _minLength(toCopy._minLength) {

}

SmoothFilter::~SmoothFilter() {
    // TODO Auto-generated destructor stub
}

std::vector<std::list<cv::Point2f> > SmoothFilter::filterFeatures(
        const std::vector<std::list<cv::Point2f> >& features) {

    std::vector<bool> toKill(features.size());
    std::vector<std::list<cv::Point2f> > result(features);
    int counter = 0;
    for (int i = 0; i < toKill.size(); ++i) {
        if (3 > features[i].size()) {
            toKill[i] = false;
        } else {
            std::list<cv::Point2f>::const_iterator it = features[i].begin();
//			cerr<<"pre"<<endl;
            cv::Point2f first = *it;
            cv::Point2f second = *(++it);
            cv::Point2f third = *(++it);
//			cv::Vec2f v1 = third - second;
//			cv::Vec2f v2 = second - first;
//			cerr<<"in "<<acos(
//                    (v1[0] * v2[0] + v1[1] * v2[1])
//                            / (cv::norm(v1) + cv::norm(v2)))/CV_PI<<endl;

            double a = cv::norm(second - third);
            double b = cv::norm(first - second);
            double c = cv::norm(third - first);

//			cerr<<"in "<<acos((b*b+c*c-a*a)/(2*b*c))/CV_PI;

            if (a > _minLength && b > _minLength
                    && _maxDeviation
                            < acos((b * b + c * c - a * a) / (2 * b * c))) {
                toKill[i] = true;
                ++counter;
//				std::cerr<<"to kill "<<counter<<" "<<i<<std::endl;
            } else {
                toKill[i] = false;
            }
//			cerr<<"post"<<endl;
        }
    }

    if (_deviantThreshold >= counter) {
        for (int j = 0; j < toKill.size(); ++j) {
            if (toKill[j]) {
               // cerr << "bug in " << j << endl;
                result.erase(result.begin() + j);
                toKill.erase(toKill.begin() + j);
                --j;
              //  cerr << "bug out" << endl;
            }
        }
    }
    return result;
}

FeatureFilter *SmoothFilter::constructCopy() const {
    SmoothFilter *copy = new SmoothFilter(*this);
    return copy;
}

