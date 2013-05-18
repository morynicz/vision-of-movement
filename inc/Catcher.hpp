/// \file
/// \brief Header file for class Cather
/// \author Michał Orynicz

#include <boost/thread.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#ifndef CATCHER_HPP
#define CATCHER_HPP

using namespace cv;

///\brief Error code for inability to open a device
static const int CATCH_CANNOT_OPEN_DEVICE = -1;
///\brief Error code for inability to open a fil
static const int CATCH_CANNOT_OPEN_FILE = -2;

///\brief Class providing ability to catch the most recent frame from 
/// video stream 

class Catcher {
        cv::Mat _fr; ///< Most recent frame
        boost::mutex *_mut;
        boost::thread *_thr;

        ///\brief Class used to ensure, that video streams buffer is empty
        class Camera {
                int _frameRate; ///< Number of frames per second
                cv::VideoCapture _cam; ///< video stream
                cv::Mat *_fr; ///< Most recent frame
                boost::mutex *_mut; ///< Mutex
            public:
                ///\brief Constructor, initializes mutex pointer
                Camera() :
                        _frameRate(0), _fr(NULL), _mut(NULL) {
                }
                ///\brief Constructor for video streams from a device
                Camera(const int &nr, cv::Mat *mat,
                        boost::mutex *mtx);
                ///\brief Constructor for video streams from a video file
                Camera(const std::string &name, cv::Mat *mat,
                        boost::mutex *mtx);
                ///\brief Function used to start emptying of the buffer
                void operator()();
                ///\brief Destructor
                ~Camera();
        };
        Camera _cam; ///< Object providing the most recent frame
    public:
        ///\brief Constructor initialising pointer fields
        Catcher();
        ///\brief initialization method for streams from a device
        void init(const int &nr);
        ///\brief initialization method for streams from a file
        void init(const std::string &name);
        ///\brief Destructor removing mutex and thread objects
        ~Catcher();
        ///\brief Method for retrieving the most recent frame
        void catchFrame(cv::Mat& frame);
        void operator>>(cv::Mat &frame) {
            catchFrame(frame);
        }

};

#endif
