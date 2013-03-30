/*
 * BirdsEyeTranslationReader.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef BIRDSEYETRANSLATIONREADER_HPP_
#define BIRDSEYETRANSLATIONREADER_HPP_

#include <TranslationReader.hpp>

class BirdsEyeTranslationReader: public TranslationReader {
	cv::Mat _homography;
	std::vector<cv::Point2f> _translations;
	unsigned int _maxFeatures;
	cv::Point2f _rotationCenter;
	std::vector<cv::Point2f> computeTranslationVectors(
			const cv::Mat &rotationMatrix);
	void filterBoarderFeatures();
	void filterFeatures();
public:
	BirdsEyeTranslationReader(const BirdsEyeTranslationReader &toCopy);
	BirdsEyeTranslationReader(const cv::Mat &homography,
			const FeatureExtractor &extractor, const FeatureTracker &tracker,
			const unsigned int& maxFeatures,
			const std::list<FeatureFilter*>& filters,
			cv::Point2f rotationCentre);
	virtual TranslationReader *constructCopy() const;
	virtual cv::Point3f readTranslation(const cv::Mat &newFrame,
			const double& rotation);
	cv::Mat getBirdsEyeView(const cv::Mat &input);
	virtual ~BirdsEyeTranslationReader();
};

#endif /* BIRDSEYETRANSLATIONREADER_HPP_ */
