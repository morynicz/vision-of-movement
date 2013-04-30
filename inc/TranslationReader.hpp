/*
 * TranslationReader.h
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef TRANSLATIONREADER_H_
#define TRANSLATIONREADER_H_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

#include "MovementReader.hpp"

#include "FeatureExtractor.hpp"
#include "FeatureTracker.hpp"
#include "FeatureFilter.hpp"

class TranslationReader: public MovementReader {
    protected:
        TranslationReader(const FeatureTracker &tracker,
                const FeatureExtractor &extractor,
                const std::list<FeatureFilter*> &filters,
                const unsigned int &maxFeatures,
                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
                        std::vector<std::list<cv::Point2f> >());
        TranslationReader(const TranslationReader& toCopy);
    public:
        virtual TranslationReader *constructCopy() const =0;
        virtual cv::Point3f readTranslation(const cv::Mat &newFrame,
                const double& rotation) =0;
        virtual ~TranslationReader();
};

#endif /* TRANSLATIONREADER_H_ */
