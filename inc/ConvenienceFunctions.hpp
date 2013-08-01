/**
 * \file
 * \date 5.04.2013
 * \author Michał Orynicz
 */

#ifndef CONVENIENCEFUNCTIONS_HPP_
#define CONVENIENCEFUNCTIONS_HPP_

#include "opencv2/core/core.hpp"
#include "Catcher.hpp"
#include "PreparationFunctions.hpp"

/// Method compares two cv::Point3f objects in x axis.
/**
 * Method checks if p1 point has lesser x value than p2
 * @param p1 - first point to compare
 * @param p2 - second point to compare
 * @return true if p1.x < p2.x
 */
bool horizontalPoint3Compare(cv::Point3f p1, cv::Point3f p2);

/// Method compares two cv::Point3f objects in y axis.
/**
 * Method checks if p1 point has lesser y value than p2
 * @param p1 - first point to compare
 * @param p2 - second point to compare
 * @return true if p1.y < p2.y
 */
bool verticalPoint3Compare(cv::Point3f p1, cv::Point3f p2);

/// Method prints info about matrix passed as argument
/**
 * Print number of channels, size, depth (type of underlying data)
 * and the matrix value, if switch set.
 * @param arg - matrix to be printed
 * @param printValues - pass true to pint matrix values
 */
void printMatrix(const cv::Mat &arg, bool printValues = false);

/// Compute mean and variance of a vector

/**
 * Function computes mean and variance from a vector of values.
 * @pre class T has to provide operators +=, * and /.
 * @param input - vector of values to calculate mean and variance
 * @param mean - calculated mean
 * @param variance - calculated variance
 */
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

/// Calculate median of a vector

/**
 * Function calculates median from values provided as vector
 * @param input - vector of values from which median shall be calculated
 * @return Median value from values provided in input.
 */
template<class T>
T median(std::vector<T> input) {
    T result = 0;
    std::nth_element(input.begin(), input.begin() + input.size() / 2,
            input.end());
    if (input.size() % 2) {
        result = (input[input.size() / 2 - 1]
                + input[input.size() / 2]) / 2;
    } else {
        result = input[input.size() / 2 - 1];
    }
    return result;
}

/// Calculate mean and variance in each axis for vector of points

/**
 * Function calculates mean and variance from vector of 3d points,
 * returning point with median values in x,y,and z axe and point
 * with variances in three axes.
 * @param input - Vector of 3d points from which statistics will be
 * calculated.
 * @param mean - point with median values from input
 * @param varinace - point with variance values from input.
 */
void pointMeanAndVariance(const std::vector<cv::Point3d> &input,
        cv::Point3d &mean, cv::Point3d &varinace);

/// Calculate median in each axis from vector of 3d points

/**
 * Function calculates median in each axis from 3d points vector
 * provided as argument
 * @param input - vector of 3d points
 * @return 3d point constining medians from values from input
 * in each dimension.
 */
cv::Point3d pointMedian(const std::vector<cv::Point3d> &input);

/// Function calculating some statistics for cv::solvePnP methods

/**
 * Function calculating statistics for cv::solvePnP methods. It is
 * not complete, so only one method can be checked without changes
 * in sourcecode. It is advised to not use until fixed.
 * @param capture - capture device
 * @param rectifyMaps - maps for distortion reduction
 * @param horizon - horizon height
 * @param deadZone - dead zone width
 * @param boardSize - size of board used for calibration
 * @param imageSize - size of image that should be received from
 * the capture device
 * @param winSize - window size parameter for cv::cornerSubPix function
 * @param zeroZone - zero zone parameter for cv::cornerSubPix function
 * @param termCrit - termination criteria parameter for cv::cornerSubPix
 * @param squareSize - length of side of chessboard square
 * @param cameraMatrix - matrix with values that model the camera
 * @param distortionCoefficients - distortion coefficients of the camera
 * @param chessboardHeight - height of the chesboard
 * @param iterations - how many iterations should be executed
 * @param mean - calculated mean position of the camera
 * (as point,mean in each dimension individually)
 * @param variance - variance of calculated camera positions (as point)
 * @param median - median of calculated camera positions (as point)
 * @param stdDev - square root of variance (as point)
 */
void checkPnPMethodStats(Catcher &capture,
        const std::vector<cv::Mat> &rectifyMaps, const int &horizon,
        const int &deadZone, const cv::Size &boardSize,
        const cv::Size &imageSize, const cv::Size &winSize,
        const cv::Size &zeroZone, const cv::TermCriteria &termCrit,
        const double &squareSize, const cv::Mat &cameraMatrix,
        const cv::Mat & distortionCoefficients,
        const double &chessboardHeight, const int & iterations,
        cv::Point3d & mean, cv::Point3d & variance,
        cv::Point3d & median, cv::Point3d &stdDev);

#endif /* CONVENIENCEFUNCTIONS_HPP_ */
