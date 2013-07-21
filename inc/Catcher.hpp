/// \file
/// \brief Header file for class Catcher
/// \author Michał Orynicz

#include <boost/thread.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#ifndef CATCHER_HPP
#define CATCHER_HPP


///\brief Error code for inability to open a device
static const int CATCH_CANNOT_OPEN_DEVICE = -1;
///\brief Error code for inability to open a fil
static const int CATCH_CANNOT_OPEN_FILE = -2;

///\brief Class providing ability to catch the most recent frame from 
/// video stream 

class Catcher: public cv::VideoCapture{
        cv::Mat _fr; ///< Most recent frame
        boost::mutex *_mut;

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
                ///\brief Method for acquiring cv::VideoCapture properties
                double get(const int &propId);
                ///\brief Method for setting cv::VideoCapture properties
                bool set(const int &propId, const double &value);
                ///\brief Method checks if cv::VideoCapture is opened
                bool isOpened();
        };
        Camera _cam; ///< Object providing the most recent frame
        boost::thread *_thr; ///< thread receiving all the images from camera

        bool retrieve(cv::Mat& image, int channel=0){return false;};
        bool grab(){return false;}
    public:
        ///\brief Constructor initialising pointer fields
        Catcher();
        ///\brief Constructor for stream from video device
        Catcher(const int &nr);
        ///\brief Constructor for stream from a video file
        Catcher(const std::string &name);
        ///\brief initialization method for streams from a device
        bool open(const int &nr);
        ///\brief initialization method for streams from a file
        bool open(const std::string &name);
        ///\brief Destructor removing mutex and thread objects
        ~Catcher();
        ///\brief Method for retrieving the most recent frame
        virtual bool read(cv::Mat& frame);
        ///\brief Method calls read method
        virtual Catcher& operator>>(cv::Mat &frame);
        ///\brief Method for acquiring cv::VideoCapture properties
        virtual double get(const int &propId);
        ///\brief Method for setting cv::VideoCapture properties
        virtual bool set(const int &propId, const double &value);
};

#endif
