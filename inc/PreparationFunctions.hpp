/**
 * \file
 * \author Michał Orynicz
 */
#ifndef PREPARATION_FUNCTIONS_HPP
#define PREPARATION_FUNCTIONS_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <list>
#include <vector>

#include "Catcher.hpp"
#include "CvTypesIo.hpp"

/**
 * Class containing parameters connected with chessboard
 * and chessboard searching.
 */
class ChessboardParameters {
    public:
        cv::Size winSize; ///< window size for subpixel corner detection
        cv::Size zeroZone; ///< not used zone for sub pixel corner detection
        float squareSize; ///< chessboard field side size
        double height; ///< chessboard height
        cv::Size size; ///< chessboard size (in squares)
        cv::TermCriteria termCrit; ///< term criteria for subpixel corners
};

/**
 * Camera parameters connected with it's global position
 * and homography for bird's eye transform
 */
class CameraSpatialParameters{
    public:
        cv::Mat homography; ///< transformation for bird's eye transform
        cv::Mat rtMatrix; ///< transformation from global to camera coordinates
        cv::Point2d rotationCenter; ///< rotation center in birds eye view
        int horizon; ///< horizon height
        int deadZone; ///< dead zone width
};

/// Manualy set horizon and dead zone using live feed from camera

/**
 * Function enables user to set manually the horizon height and deadZone width
 * using live images from camera, with two sliders and couple of image windows.
 * @param[in] capture - source of images
 * @param[in] rectifyMaps - maps for distortion removal
 * @param[out] horizon - horizon height
 * @param[out] deadZone - dead zone width (where no features will be searched
 * for)
 * @param[in] imageSize - desired size of images from camera
 */
void calibrateParameters(cv::VideoCapture &capture,
        std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
        const cv::Size &imageSize);

/// Manualy set horizon and dead zone using single image form images source

/**
 * Function enables user to set manually the horizon height and deadZone width
 * using single image from images source, with two sliders and couple of image
 * windows.
 * @param[in] capture - source of images
 * @param[in] rectifyMaps - maps for distortion removal
 * @param[out] horizon - horizon height
 * @param[out] deadZone - dead zone width (where no features will be searched
 * for)
 * @param[in] imageSize - desired size of images from camera
 */
void calibrateParametersSingleImage(cv::VideoCapture &capture,
        std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
        const cv::Size &imageSize);

/// Function searches for chessboard corners subpixel positions on undistorted
/// image

/**
 * Function performs a search of chessboard corners in lower parts of a series
 * of images taken form the source, and returns their positions with subpixel
 * precission when all are found.
 * \see cv::cornersSubPix
 * @param capture - images source
 * @param rectificationMaps - maps used to undistort image
 * @param horizon - horizon height
 * @param deadZone - dead zone width
 * @param boardSize - chessboard size
 * @param imageSize - desired image size
 * @param winSize - window size for subpixel precission calculations
 * @param zeroZone - excluded area for subpixel precission calculations
 * @param termCriteria - termination criteria for subpixel precission calculations
 * @return
 */
std::vector<cv::Point2f> getChessboardCorners(cv::VideoCapture &capture,
        const std::vector<cv::Mat> &rectificationMaps,
        const int &horizon, const int &deadZone,
        const cv::Size &boardSize, const cv::Size &imageSize,
        const cv::Size &winSize, const cv::Size &zeroZone,
        const cv::TermCriteria &termCriteria);

/// Function calculates camera position and orienatation

/**
 * Function returns camera orientation and position in world coordinates, and
 * rotation center in bird's eye view. Also calculates bird's eye transform
 * matrix.
 * @param[in] corners - chessboard corners position
 * @param[in] imageSize - input image desired size
 * @param[in] boardSize - chessboard size [squares]
 * @param[in] squareSize - chessboard single square side length [mm].
 * @param[in] horizon - horizon height
 * @param[in] deadZone - dead zone width
 * @param[in] cameraMatrix - camera model matrix (intrinsics)
 * @param[in] distortionCoefficients - camera distortion model coefficients
 * @param[in] chessboardHeight - chessboard height
 * @param[out] homography - bird's eye view transform matrix
 * @param[out] rotationCenter - rotation center in bird's eye view
 * @param[out] rtMatrix - transform from world (chessbard) coordiantes to
 * camera coordiantes
 */
void getHomographyRtMatrixAndRotationCenter(
        std::vector<cv::Point2f> corners, const cv::Size& imageSize,
        const cv::Size& boardSize, const double& squareSize,
        const int &horizon, const int &deadZone,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distortionCoefficients,
        const double &chessboardHeight, cv::Mat& homography,
        cv::Point2d &rotationCenter, cv::Mat &rtMatrix);

/// Assemble homogenous coordinates transform matrix

/**
 * Assemble homogenous 4x4 transform matrix describing rotation and
 * translation, from rotation and translation vectors
 * @param rot - rotation vector
 * @param trans - translation vector
 * @return Homogenous coordinates 4x4 transform matrix in form
 * \f[[R T;0 1]\f].
 */
cv::Mat rtTransformMatrix(const cv::Mat &rot, const cv::Mat &trans);

/// Assemble inverted homogenous coordinates transform matrix

/**
 * Assemble inverted homogenous 4x4 transform matrix describing rotation and
 * translation, from rotation and translation vectors
 * @param rot - rotation vector
 * @param trans - translation vector
 * @return Inverted homogenous coordinates 4x4 transform matrix in form
 * \f[[R^t -R^tT;0 1]\f].
 */
cv::Mat invRtTransformMatrix(const cv::Mat& rot,
        const cv::Mat& trans);


/// Invert the homogenous coordinates transform matrix

/**
 * Function inverts homogenous coordinates transformation matrix
 * @param rtTransformMatrix - homogenous coordinates transformation matrix
 * @return inverted homogenous coordinates transformation matrix
 */
cv::Mat invRtTransformMatrix(const cv::Mat& rtTransformMatrix);

/**
 * Read camera internal and distortion parameters from YAML file
 * @param[in] fileName - source YAML file name
 * @param[out] undistortionMaps - maps used for undistorting images
 * @param[out] cameraMatrix - camera intrinsics matrix
 * @param[out] distortionCoefficients - camera distortion parameters
 * @param[out] imageSize - output image size
 */
void getCameraParameters(const std::string &fileName,
        std::vector<cv::Mat> &undistortionMaps, cv::Mat& cameraMatrix,
        cv::Mat& distortionCoefficients, cv::Size &imageSize);

/// Function enables use storing ChessboardParameters with OpenCV persistence

/**
 * Function enables use storing ChessboardParameters with OpenCV persistence
 * @param[in] fs - file storage
 * @param[in] chessboard - chessboard parameters
 * @return file storage stream reference
 */
cv::FileStorage &operator<<(cv::FileStorage &fs,
        const ChessboardParameters &chessboard);

/// Function enables use recovery ChessboardParameters from OpenCV persistence

/**
 * Function enables use recovery ChessboardParameters from OpenCV persistence
 * @param node - file node
 * @param chessboard - Chessboard parameters
 */
void operator>>(const cv::FileNode &node,
        ChessboardParameters &chessboard);

/// Function enables use storing CameraSpatialParameters with OpenCV
/// persistence

/**
 * Function enables use storing CameraSpatialParameters with OpenCV persistence
 * @param[in] fs - file storage
 * @param[in] camera - CameraSpatialParameters
 * @return file storage stream reference
 */

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const CameraSpatialParameters &camera);

/// Function enables use recovery CameraSpatialParameters from OpenCV persistence

/**
 * Function enables use recovery CameraSpatialParameters from OpenCV persistence
 * @param node - file node
 * @param camera - CameraSpatialParameters
 */

void operator>>(const cv::FileNode &node,
        CameraSpatialParameters &camera);

#endif
