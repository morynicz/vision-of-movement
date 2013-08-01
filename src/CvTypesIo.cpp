/**
 * \file
 *  \date  20.05.2013
 *  \author Michał Orynicz
 */

#include "CvTypesIo.hpp"

void operator>>(const cv::FileNode &node, cv::Size &size) {
    node[0] >> size.width;
    node[1] >> size.height;
}

void operator>>(const cv::FileNode &node,
        cv::TermCriteria &termCrit) {
    node["type"] >> termCrit.type;
    node["maxCount"] >> termCrit.maxCount;
    node["epsilon"] >> termCrit.epsilon;
}

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const cv::TermCriteria &termCrit) {
    fs << "{" << "type" << termCrit.type << "maxCount"
            << termCrit.maxCount << "epsilon" << termCrit.epsilon
            << "}";
    return fs;
}
