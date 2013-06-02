/*
 * SmoothFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "SmoothFilter.hpp"
#include <cmath>
#include <algorithm>

//#include<iostream>
using namespace std;

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
//			cerr<<"pre"<<endl;
//            cv::Point2f first = *it;
//            cv::Point2f second = *(++it);
//            cv::Point2f third = *(++it);
            cv::Point2f zeroth = *it;
            cv::Point2f first = *(++it);
            ++it;
            cv::Point2f third = *(++it);
            ++it;
            ++it;
            ++it;
            cv::Point2f seventh = *(++it);
//			cv::Vec2f v1 = third - second;
//			cv::Vec2f v2 = second - first;
//			cerr<<"in "<<acos(
//                    (v1[0] * v2[0] + v1[1] * v2[1])
//                            / (cv::norm(v1) + cv::norm(v2)))/CV_PI<<endl;

//            double a = cv::norm(second - third);
//            double b = cv::norm(first - second);
//            double c = cv::norm(third - first);

//			cerr<<"in "<<acos((b*b+c*c-a*a)/(2*b*c))/CV_PI;

            cv::Vec2f v10 = first - zeroth;
            cv::Vec2f v31 = third - first;
            cv::Vec2f v73 = seventh - third;

            double cos1031 = v10.dot(v31)
                    / (cv::norm(v10) * cv::norm(v31)) * 180/CV_PI;
            double cos3173 = v31.dot(v73)
                    / (cv::norm(v31) * cv::norm(v73)) * 180/CV_PI;
            double cos1073 = v10.dot(v73)
                    / (cv::norm(v10) * cv::norm(v73)) * 180/CV_PI;

//            std::cerr << zeroth << std::endl;
//            std::cerr << first << std::endl;
//            std::cerr << third << std::endl;
//            std::cerr << seventh << std::endl;
//
//            std::cerr<<cv::norm(v10 )<<std::endl;
//            std::cerr<<cv::norm(v31 )<<std::endl;
//            std::cerr<<cv::norm(v73 )<<std::endl;
//
//            std::cerr << cos1031 << std::endl;
//            std::cerr << cos3173 << std::endl;
//            std::cerr << cos1073 << std::endl;
//            if (a > _minLength && b > _minLength
//                    && (_maxDeviation
//                            < acos(
//                                    (b * b + c * c - a * a)
//                                            / (2 * b * c))
//                            || _maxLength < a + b)) {
            if (_maxDeviation < abs(cos1031) || _maxDeviation < abs(cos1073)
                    || _maxDeviation < abs(cos3173)) {
                toKill[i] = true;
                ++counter;
//				std::cerr<<"to kill "<<counter<<" "<<i<<std::endl;
            } else {
                toKill[i] = false;
            }
//			cerr<<"post"<<endl;
        }
    }

    if (_deviantThreshold >= counter / (double) features.size()) {
        for (unsigned int j = 0; j < toKill.size(); ++j) {
            if (toKill[j]) {
                // cerr << "bug in " << j << endl;
                result.erase(result.begin() + j);
                toKill.erase(toKill.begin() + j);
                --j;
                //  cerr << "bug out" << endl;
            }
        }
    }else{
  //      std::cerr<<"deviant overflow"<<std::endl;
    }
    return result;
}

FeatureFilter *SmoothFilter::constructCopy() const {
    SmoothFilter *copy = new SmoothFilter(*this);
    return copy;
}

