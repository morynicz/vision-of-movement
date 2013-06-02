/*
 * CvTypesIo.hpp
 *
 *  Created on: May 20, 2013
 *      Author: link
 */

#ifndef CVTYPESIO_HPP_
#define CVTYPESIO_HPP_

#include <opencv2/core/core.hpp>

void operator>>(const cv::FileNode &node, cv::Size &size);

void operator>>(const cv::FileNode &node,
        cv::TermCriteria &termCrit);
template<class T>
void operator>>(const cv::FileNode &node,
        cv::Point_<T> &point){
        node[0]>>point.x;
        node[1]>>point.y;
}

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const cv::TermCriteria &termCrit);

#endif /* CVTYPESIO_HPP_ */
