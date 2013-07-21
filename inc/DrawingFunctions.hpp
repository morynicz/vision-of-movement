/**
 * \file DrawingFunctions.hpp
 *
 * \date 5.04.2013
 * \author Michał Orynicz
 */

#ifndef DRAWINGFUNCTIONS_HPP_
#define DRAWINGFUNCTIONS_HPP_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <list>
#include <vector>

/// Function draws horizon with "death zone separaring two parts of image

/**
 * Draw on the image a horizontal area 2 x deadZone wide with center at horizon
 * height constrained by two green lines.
 * @param[in,out] image - image on which the lines will be drawn
 * @param[in] horizon - height of the horizon plane
 * @param[in] deadZone - width/2 of the safety area around horizon
 */
void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
        const int& deadZone);

/// Function draws 2d route indicated by route points, and current orientation

/**
 * Function draws route indicated by x and y coordinates of points in route
 * list, and marks the current position as a big triangle oriented as indicated
 * by z coordinate of last point on route (orientation)
 * @param route - list of 3d points containing x,y coordinates and orientation as
 * z coordinate
 * @return A matrix with route drawn
 */
cv::Mat drawTraveledRoute(const std::list<cv::Point3f> &route);


/// Function draws current position of features and their previous positions

/**
 * Function draws current positions of features contained in featureHistory as dots
 * and their previous positions as lines originating from current position
 * @param[in,out] newImage - image on which features will be drawn
 * @param[in] featureHistory - container with present and past positions of features
 * @param[in] radius - radius of disk indicating current position of a feature
 */
void drawFeatureHistory(cv::Mat &newImage,
        const std::vector<std::list<cv::Point2f> > &featureHistory,
        const int &radius);



/// Function draws current position of features and their previous positions,
/// after coordinates transformation

/**
 * Function draws current positions of features contained in featureHistory as dots
 * and their previous positions as lines originating from current position, after
 * coordinates transformation
 * @param[in,out] newImage - image on which features will be drawn
 * @param[in] featureHistory - container with present and past positions of features
 * @param[in] transformMatrix - transformation for feature coordinates
 * @param[in] radius - radius of disk indicating current position of a feature
 */
void drawFeatureHistory(cv::Mat &newImage,
        const std::vector<std::list<cv::Point2f> > &featureHistory,
        const cv::Mat &transformMatrix, const int &radius);


/// Function draws feature history in upper and lower part of image

/**
 * Function draws transformed features on upper and lower area of image.
 * @param[in,out] image - image on which features will be drawn.
 * @param upperFeatures - tracked features with history for upper part of image
 * @param upperTransform - coordinates transform for upper features
 * @param lowerFeatures - tracked features with history for lower part of image
 * @param lowerTransform - coordinates transform for lower features
 * @param horizon - height on image at which horizon is located
 * @param deadZone - safety zone width/2 around horizon
 * @param radius - radius of disks indicating current position of features
 */
void drawFeaturesUpperAndLower(cv::Mat &image,
        const std::vector<std::list<cv::Point2f> > &upperFeatures,
        const cv::Mat &upperTransform,
        const std::vector<std::list<cv::Point2f> > &lowerFeatures,
        const cv::Mat &lowerTransform, const int &horizon,
        const int & deadZone, const int &radius=5);
#endif /* DRAWINGFUNCTIONS_HPP_ */
