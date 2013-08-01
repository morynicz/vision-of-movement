/*
 * DrawingFunctions.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */
#include "DrawingFunctions.hpp"
#include "ErrorCodes.hpp"
#include "ConvenienceFunctions.hpp"

#include <iostream>
void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
        const int& deadZone) {
    cv::line(image, cv::Point2f(0, horizon + deadZone),
            cv::Point2f(image.cols, horizon + deadZone),
            CV_RGB(0,255,0), 2, CV_AA);
    cv::line(image, cv::Point2f(0, horizon - deadZone),
            cv::Point2f(image.cols, horizon - deadZone),
            CV_RGB(0,255,0), 2, CV_AA);
}

cv::Mat drawTraveledRoute(const std::list<cv::Point3f> &route) {
    cv::Mat result;
    cv::Size mapSize(1000, 1000);
    int scale = 0.1;

    if (0 == mapSize.width) {
        mapSize.width = 1;
    }
    if (0 == mapSize.height) {
        mapSize.height = 1;
    }

    std::list<cv::Point3f>::const_iterator bg = route.begin();
    std::list<cv::Point3f>::const_iterator end = route.begin();
    cv::Point3f center(mapSize.width / 2, mapSize.height / 2, 0);

    result = cv::Mat(mapSize, CV_8UC3, cv::Scalar::all(255));

    for (++end; route.end() != end; ++end, ++bg) {
        cv::Point2f start((bg->x) * scale + center.x,
                (bg->y) * scale + center.y);
        cv::Point2f finish((end->x) * scale + center.x,
                (end->y) * scale + center.y);
        cv::line(result, start, finish, CV_RGB(0,0,255), 10, 8);

    }

    cv::Point3f curr = route.back() * scale + center;
    cv::Point2f curr2d(curr.x, curr.y);
    std::vector<cv::Point2f> preTrans;
    cv::Mat postTrans;
    preTrans.push_back(
            cv::Point2f(-0.03 * mapSize.width, 0) + curr2d);
    preTrans.push_back(cv::Point2f(0.03 * mapSize.width, 0) + curr2d);
    preTrans.push_back(cv::Point2f(0, 0.1 * mapSize.width) + curr2d);
    preTrans.push_back(
            cv::Point2f(-0.03 * mapSize.width, 0) + curr2d);

    cv::Mat rotMat = cv::getRotationMatrix2D(
            cv::Point2f(curr.x, curr.y), curr.z / scale * 180 / CV_PI,
            1);

    cv::transform(preTrans, postTrans, rotMat);
    cv::Mat out;
    postTrans.convertTo(out, CV_32S);
    cv::fillConvexPoly(result, out, CV_RGB(255,0,0), 1, 0);

    return result;
}

void drawFeatureHistory(cv::Mat &newImage,
        const std::vector<std::list<cv::Point2f> > &featureHistory,
        const int &radius) {
    for (unsigned int i = 0; i < featureHistory.size(); ++i) {
        cv::circle(newImage, featureHistory[i].front(), radius,
                cv::Scalar::all(255), -1, 8, 0);
        std::list<cv::Point2f>::const_iterator itb =
                featureHistory[i].begin();
        std::list<cv::Point2f>::const_iterator ite = itb;
        if (!featureHistory.size() < 2) {
            for (ite++; ite != featureHistory[i].end();
                    ++ite, ++itb) {
                cv::line(newImage, *ite, *itb, cv::Scalar::all(255),
                        1, CV_AA);
            }
        }
    }
}

std::vector<std::list<cv::Point2f> > transformFeatureHistory(
        const std::vector<std::list<cv::Point2f> > &input,
        const cv::Mat &transformMatrix) {
    if (!transformMatrix.empty()) {
        std::vector<std::list<cv::Point2f> > result(input.size());
        for (unsigned int i = 0; i < input.size(); ++i) {
            std::vector<cv::Point2f> tmp(input[i].begin(),
                    input[i].end());
            std::vector<cv::Point2f> out;
            perspectiveTransform(tmp, out, transformMatrix);
            result[i] = std::list<cv::Point2f>(out.begin(),
                    out.end());
        }
        return result;
    } else {
        return input;
    }
}

void drawFeatureHistory(cv::Mat &newImage,
        const std::vector<std::list<cv::Point2f> > &featureHistory,
        const cv::Mat &transformMatrix, const int &radius) {
    drawFeatureHistory(newImage,
            transformFeatureHistory(featureHistory, transformMatrix),
            radius);
}

void drawFeaturesUpperAndLower(cv::Mat &image,
        const std::vector<std::list<cv::Point2f> > &upperFeatures,
        const cv::Mat &upperTransform,
        const std::vector<std::list<cv::Point2f> > &lowerFeatures,
        const cv::Mat &lowerTransform, const int &horizon,
        const int & deadZone, const int &radius) {
    cv::Rect lowerRoi(cv::Point2f(0, horizon + deadZone),
            cv::Size(image.cols, image.rows - horizon - deadZone));
    cv::Rect upperRoi(cv::Point2f(0, 0),
            cv::Size(image.cols, horizon - deadZone));
    cv::Mat upper = image(upperRoi);
    cv::Mat lower = image(lowerRoi);
    drawFeatureHistory(lower, lowerFeatures, lowerTransform, radius);
    drawFeatureHistory(upper, upperFeatures, upperTransform, radius);
}
