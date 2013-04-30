/*
 * DrawingFunctions.hpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */

#ifndef DRAWINGFUNCTIONS_HPP_
#define DRAWINGFUNCTIONS_HPP_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <list>
#include <vector>

void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
        const int& deadZone);

cv::Mat drawTraveledRoute(const std::list<cv::Point3f> &route);

void drawFeatureHistory(cv::Mat &newImage,
        const std::vector<std::list<cv::Point2f> > &featureHistory,
        const int &radius);

void drawFeatureHistory(cv::Mat &newImage,
        const std::list<cv::Point2f> &featureHistory,
        const cv::Mat &transformMatrix, const int &radius);

void drawFeaturesUpperAndLower(cv::Mat &image,
        const std::vector<std::list<cv::Point2f> > &upperFeatures,
        const cv::Mat &upperTransform,
        const std::vector<std::list<cv::Point2f> > &lowerFeatures,
        const cv::Mat &lowerTransform, const int &horizon,
        const int & deadZone, const int &radius=5);
#endif /* DRAWINGFUNCTIONS_HPP_ */
