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
#endif /* CONVENIENCEFUNCTIONS_HPP_ */
