/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef SHITHOMASIFEATUREEXTRACTOR_HPP_
#define SHITHOMASIFEATUREEXTRACTOR_HPP_

#include "FeatureExtractor.hpp"
#include "CvTypesIo.hpp"

/// Class extracting Shi - Thomasi corners from an image
/**
 * Uses cv::goodFeaturesToTrack and cv::cornerSubPix
 */
class ShiThomasiFeatureExtractor: public FeatureExtractor {
        /**
         * Threshold value for max eigenvalues of corners found.
         * Corners with max eigenvalue lower than
         * biggest_eigenvalue_found * _qualityLevel will be discarded
         */
        double _qualityLevel;
        /**
         * Minimum euclidean distance between returned corners
         */
        double _minDistance;
        /**
         * Size of averaging block used to compute image derivative
         */
        int _blockSize;
        /**
         * Half of the size of search window for subPix calculations
         */
        cv::Size _winSize;
        /**
         * Size of safety zone for subPix calculations
         */
        cv::Size _zeroZone;
        /**
         * Term criteria for subPix calculations
         */
        cv::TermCriteria _termCrit;
        /// Default constructor
        ShiThomasiFeatureExtractor();
    public:
        /**
         * Constructor
         * @param qualityLevel - thresholding value deciding if corner found
         * will be rejected \see ShiThomasiFeatureExtractor::_qualityLevel
         * @param minDistance - minimum euclidean distance between detected
         * corners
         * @param blockSize - size of averaging block used to calculate
         * image derivatives
         * @param winSize - window size for subpixel accuracy calculations
         * @param zeroZone - safety zone for subpixel accuracy calculations
         * @param termCrit - termination criteria for subpixel accuracy
         * calculations
         */
        ShiThomasiFeatureExtractor(const double &qualityLevel,
                const double &minDistance, const int &blockSize,
                const cv::Size &winSize, const cv::Size &zeroZone,
                const cv::TermCriteria &termCrit);
        /**
         * Copy constructor
         * @param toCopy - object which values will be used to initialise
         * this object
         */
        ShiThomasiFeatureExtractor(
                const ShiThomasiFeatureExtractor &toCopy);
        /// Method creates a pointer to a copy of this object
        virtual FeatureExtractor *constructCopy() const;
        /// Method extracts features from given image

        /**
         * Method extracts features from the given image
         * @param[in] input - image from which features will be extracted
         * @param[in] maxCorners - max features to be extracted
         * @return vector of 2d points indicating locations of the features
         */
        virtual std::vector<cv::Point2f> extractFeatures(
                const cv::Mat &input, const int & maxCorners) const;
        virtual ~ShiThomasiFeatureExtractor();
};

/// Small helper class containing all parameter values needed for
/// ShiThomasiExtractor initialisation
class ShiThomasiParameters {
    public:
        /**
         * Threshold value for max eigenvalues of corners found.
         * Corners with max eigenvalue lower than
         * biggest_eigenvalue_found * _qualityLevel will be discarded
         */
        double qualityLevel;
        /**
         * Minimum euclidean distance between returned corners
         */
        double minDistance;
        /**
         * Size of averaging block used to compute image derivative
         */
        int blockSize;
        /**
         * Half of the size of search window for subPix calculations
         */
        cv::Size winSize;
        /**
         * Size of safety zone for subPix calculations
         */
        cv::Size zeroZone;
        /**
         * Term criteria for subPix calculations
         */
        cv::TermCriteria termCrit;

};

/**
 * Function writes a ShiThomasiParameters structure
 * to a file storage
 * @param[in,out] fs - file storage
 * @param[in] shiThomasi - structure to be stored
 * @return fs file storage for chain calls
 */
cv::FileStorage &operator<<(cv::FileStorage &fs,
        const ShiThomasiParameters &shiThomasi);


/**
 * Function reads ShiThomasiParameters structure from a file
 * storage
 * @param[in] node - file storage node containing ShiThomasiParameters
 * object
 * @param[out] shiThomasi - structure with values read from file storage
 */
void operator>>(const cv::FileNode &node,
        ShiThomasiParameters &shiThomasi);

#endif /* SHITHOMASFEATUREEXTRACTOR_HPP_ */
