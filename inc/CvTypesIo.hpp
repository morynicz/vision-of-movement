/**
 * \file
 * \date 20.03.2013
 * \author Michał Orynicz
 */

#ifndef CVTYPESIO_HPP_
#define CVTYPESIO_HPP_

#include <opencv2/core/core.hpp>

/// Operator enabling to use cv::Size with OpenCV persistence
/// functionality (reading)

/**
 * Function provides a way to read a cv::Size value from
 * OpenCV text file.
 * @param[in] node - file node in which the size value is stored
 * @param[out] size - value read
 */
void operator>>(const cv::FileNode &node, cv::Size &size);

/// Operator enabling to use cv::TermCriteria with OpenCV persistence
/// functionality (reading)

/**
 * Function provides a way to read a cv::TermCriteria value from
 * OpenCV text file.
 * @param[in] node - file node in which the termCrit value is stored
 * @param[out] termCrit - value read
 */
void operator>>(const cv::FileNode &node, cv::TermCriteria &termCrit);

/// Operator enabling to use cv::Point_<T> with OpenCV persistence
/// functionality (reading)

/**
 * Function provides a way to read a cv::Point_<T> value from
 * OpenCV text file.
 * @param[in] node - file node in which the size value is stored
 * @param[out] point - value read
 */
template<class T>
void operator>>(const cv::FileNode &node, cv::Point_<T> &point) {
    node[0] >> point.x;
    node[1] >> point.y;
}

/// Operator enabling to use cv::TermCriteria with OpenCV persistence
/// functionality (writing)

/**
 * Function provides a way to store a cv::TermCriteria value in
 * OpenCV text file.
 * @param[in] fs - file storage in which the size value is stored
 * @param[in] termCrit - value to be stored
 * @return fs file storage to enable chain calls.
 */

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const cv::TermCriteria &termCrit);

#endif /* CVTYPESIO_HPP_ */
