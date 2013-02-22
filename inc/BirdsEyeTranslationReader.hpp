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
public:
	BirdsEyeTranslationReader(const BirdsEyeTranslationReader &toCopy);
	BirdsEyeTranslationReader(const cv::Mat &homography,const FeatureTracker &tracker);
	virtual TranslationReader *constructCopy() const;
	virtual cv::Point2f readTranslation(const cv::Mat &newFrame);
	cv::Mat getBirdsEyeView(const cv::Mat &input);
	virtual ~BirdsEyeTranslationReader();
};

#endif /* BIRDSEYETRANSLATIONREADER_HPP_ */
