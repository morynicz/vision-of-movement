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
#include <opencv2/calib3d/calib3d.hpp>

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
        const double &margin, const cv::Size &viewSize,
        const std::vector<std::list<cv::Point2f> > &trackedFeatures) :
        TranslationReader(tracker, extractor, filters, maxFeatures,
                trackedFeatures), _rotationCenter(rotationCentre), _viewSize(
                viewSize), _homography(homography.clone()) {
    _filters.push_front(
            new ImageEdgeFilter(homography, imageSize, margin));
    temp.clear();

}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
        const BirdsEyeTranslationReader &toCopy) :
        TranslationReader(toCopy), _translations(
                toCopy._translations), _rotationCenter(
                toCopy._rotationCenter), _viewSize(toCopy._viewSize), _homography(
                toCopy._homography) {
    temp.clear();
}

BirdsEyeTranslationReader::~BirdsEyeTranslationReader() {
}

TranslationReader *BirdsEyeTranslationReader::constructCopy() const {
    return new BirdsEyeTranslationReader(*this);
}

static int bla = 0;

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
//        std::vector<cv::Point2f> corners;
//        cv::Mat show = newTransformed.clone();
//        ++bla;
//        if (bla > 10) {
//            bla = 0;
//            bool found = cv::findChessboardCorners(newTransformed,
//                    cv::Size(9, 6), corners,
//                    CV_CALIB_CB_FAST_CHECK
//                            | CV_CALIB_CB_ADAPTIVE_THRESH
//                    /*| CV_CALIB_CB_NORMALIZE_IMAGE*/);
//            drawChessboardCorners(show, cv::Size(9, 6), corners,
//                    found);
//            if (found) {
//                std::cerr << "found" << std::endl;
//                temp.push_front(corners);
//                if (3 < temp.size()) {
//                    temp.pop_back();
//                }
//
//                if (3 == temp.size()) {
//                    std::list<std::vector<cv::Point2f> >::iterator iter1 =
//                            temp.begin();
//                    std::list<std::vector<cv::Point2f> >::iterator iter2 =
//                            temp.begin();
//                    std::list<std::vector<cv::Point2f> >::iterator iter3 =
//                            temp.begin();
//                    iter2++;
//                    iter3++;
//                    iter3++;
//
//                    double sumX = 0;
//                    double sumY = 0;
//                    unsigned int counter = 0;
//
//                    std::vector<double> xs, ys;
//                    xs.resize(temp.front().size());
//                    ys.resize(temp.front().size());
//                    for (unsigned int i = 0;
//                            i < temp.front().size() - 1; ++i) {
//
//                        double x0, x1, x2, x3, y0, y1, y2, y3;
//
//                        x1 = iter1->at(i).x;
//                        x2 = iter2->at(i).x;
//                        x3 = iter3->at(i).x;
//                        y1 = iter1->at(i).y;
//                        y2 = iter2->at(i).y;
//                        y3 = iter3->at(i).y;
//
////                    x1=8;x2=7;x3=6;
////                    y1=4;y2=7;y3=8;
//                        if (x2 == x1 || x2 == x3 || y1 == y2
//                                || y2 == y3) {
//                            continue;
//                        }
//                        double a1 = -(x2 - x1) / (y2 - y1);
//                        double a2 = -(x3 - x2) / (y3 - y2);
//                        double b1 = (y2 + y1) / 2
//                                - a1 * (x2 + x1) / 2;
//                        double b2 = (y3 + y2) / 2
//                                - a2 * (x3 + x2) / 2;
//
//                        if (a1 == a2) {
//                            continue;
//                        }
//
//                        x0 = (b2 - b1) / (a1 - a2);
//                        y0 = a1 * x0 + b1;
//
////                    std::cerr<<y0<<" "<<a2*x0+b2<<std::endl;
//
//                        sumX += x0;
//                        sumY += y0;
//                        ++counter;
//                        xs[i] = x0;
//                        ys[i] = y0;
//                        std::cerr << i << std::endl;
//                        std::cerr << cv::Point2f(x1, y1) << std::endl;
//                        std::cerr << cv::Point2f(x2, y2) << std::endl;
//                        std::cerr << cv::Point2f(x3, y3) << std::endl;
//                        std::cerr << std::endl;
//                        std::cerr << cv::Point2f(x0, y0) << std::endl;
//                        std::cerr << std::endl;
//                        cv::circle(show, cv::Point2d(x0, y0), 4,
//                                cv::Scalar::all(255), 3, 8);
//
//                    }
//                    unsigned int xH = xs.size() / 2;
//                    unsigned int yH = ys.size() / 2;
//                    std::nth_element(xs.begin(), xs.begin() + xH,
//                            xs.end());
//                    std::nth_element(ys.begin(), ys.begin() + yH,
//                            ys.end());
//                    std::cerr << cv::Point2d(xs[xH], ys[yH])
//                            << std::endl;
//                    std::cerr
//                            << cv::Point2d(sumX / counter,
//                                    sumY / counter) << std::endl;
//                    std::cerr << _rotationCenter << std::endl;
////                cv::circle(show,cv::Point2d(xs[xH], ys[yH]),8,cv::Scalar::all(255),3,8);
////                cv::circle(show,cv::Point2d(sumX/counter, sumY/counter),4,cv::Scalar::all(255),3,8);
//                    double medx = sumX / counter;
//                    double medy = sumX / counter;
//                    double varx = 0;
//                    double vary = 0;
//                    for (unsigned k = 0; k < xs.size(); ++k) {
//                        varx += (medx - xs[k]) * (medx - xs[k]);
//                        vary += (medy - ys[k]) * (medy - ys[k]);
//                    }
//
//                    std::cerr << varx / xs.size() << std::endl;
//                    std::cerr << vary / ys.size() << std::endl;
//
//                }
//            }
//            cv::imshow("c3", show);
//        }
        if (!_trackedFeatures.empty()) {
//			cv::Mat pre = drawFeatureHistory(newTransformed, _trackedFeatures);
//			imshow("c1", pre);
//			imshow("ol", _oldFrame);
            for (std::list<FeatureFilter*>::iterator it =
                    _filters.begin(); it != _filters.end(); ++it) {
                _trackedFeatures = (*it)->filterFeatures(
                        _trackedFeatures);
            }
//			cv::Mat post = newTransformed.clone();
//			drawFeatureHistory(post, _trackedFeatures,3);
//            cv::imshow("c1", post);
            if (!_trackedFeatures.empty()) {
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
//
//    cv::Mat newRot;
//        cv::warpAffine(newTransformed, newRot, rotationMatrix,
//                newTransformed.size());
//        imshow("c2", _oldFrame - newRot);
//      imshow("c3", newRot);
// cv::waitKey(1);

    }
    _oldFrame = newTransformed.clone();
    // std::cerr << "bird out" << std::endl;
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

