/*
 * ConvenienceFunctions.hpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */

#ifndef CONVENIENCEFUNCTIONS_HPP_
#define CONVENIENCEFUNCTIONS_HPP_

#include "opencv2/core/core.hpp"

bool horizontalPoint3Compare(cv::Point3f p1, cv::Point3f p2);

bool verticalPoint3Compare(cv::Point3f p1, cv::Point3f p2);

void printMatrix(const cv::Mat &arg, bool printValues = false);

template<class T>
void meanAndVariance(const std::vector<T> &input, T &mean,
        T &variance) {
    T sum = 0;
    for (unsigned int i = 0; i < input.size(); ++i) {
        sum += input[i];
    }
    mean = sum / input.size();
    sum = 0;
    for (unsigned int i = 0; i < input.size(); ++i) {
        sum += (mean - input[i]) * (mean - input[i]);
    }
    variance = sum / input.size();
}

template<class T>
T median(std::vector<T> input) {
    T result = 0;
    std::nth_element(input.begin(), input.begin() + input.size() / 2,
            input.end());
    if (input.size() % 2) {
        result = (input[input.size() / 2 - 1] + input[input.size()/2])
                / 2;
    } else {
        result = input[input.size()/2 - 1];
    }
    return result;
}

void pointMeanAndVariance(const std::vector<cv::Point3d> &input,
        cv::Point3d &mean, cv::Point3d &varinace);

cv::Point3d pointMedian(const std::vector<cv::Point3d> &input);

#endif /* CONVENIENCEFUNCTIONS_HPP_ */
