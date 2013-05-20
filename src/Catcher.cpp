///\file
///\brief File containing implementation of class Catcher
///\author Michał Orynicz
#include "Catcher.hpp"

#include <iostream>

using namespace std;

/// Constructor for video streams originating from a device
/// \param nr - number of the device
/// \param mat - pointer to cv::Mat object which will contain the most
/// recent frame
/// \param mtx - pointer to mutex 
Catcher::Camera::Camera(const int &nr, cv::Mat *mat,
        boost::mutex *mtx) :
        _frameRate(0), _cam(nr), _fr(mat), _mut(mtx) {

    if (!_cam.isOpened()) {
        cv::Exception err(CATCH_CANNOT_OPEN_DEVICE,
                "file cannot be opened", __func__, __FILE__,
                __LINE__);
        throw err;
    }
}

/// Constructor for video streams originating from a device
/// \param name - name of video file
/// \param mat - pointer to cv::Mat object which will contain the most
/// recent frame
/// \param mtx - pointer to mutex 
Catcher::Camera::Camera(const std::string &name, cv::Mat *mat,
        boost::mutex *mtx) :
        _cam(name), _fr(mat), _mut(mtx) {

    if (!_cam.isOpened()) {
        cv::Exception err(CATCH_CANNOT_OPEN_FILE,
                "file cannot be opened", __func__, __FILE__,
                __LINE__);
        throw err;
    }
    _frameRate = _cam.get(CV_CAP_PROP_FPS);
    if (_frameRate <= 0) {
        _frameRate = 30;
    }
}

/// Empty destructor
Catcher::Camera::~Camera() {
}

/// Method which reads subsequent frames from stream. Ensures that frame 
/// buffer of stream is empty
void Catcher::Camera::operator()() {
    if (_frameRate > 0) {
        while (!boost::this_thread::interruption_requested()) {
            _mut->lock();
            _cam >> (*_fr);
            _mut->unlock();
            unsigned char time = 10;
            boost::this_thread::sleep(
                    boost::posix_time::millisec(10));
        }
    } else {
        while (!boost::this_thread::interruption_requested()) {
            _mut->lock();
            _cam >> (*_fr);
            _mut->unlock();
            boost::this_thread::sleep(
                    boost::posix_time::millisec(10));
        }
    }
}

///Method for acquiring cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::get documentation
double Catcher::Camera::get(const int &propId) {
    return _cam.get(propId);
}
///Method for setting cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::set documentation
///\param value - value to which property @propId will be set
bool Catcher::Camera::set(const int &propId, const double &value) {
    return _cam.set(propId, value);
}

/// Initializes mut and thr pointers
Catcher::Catcher() :
        _mut(NULL), _thr(NULL) {
    _mut = NULL;
    _thr = NULL;
}

/// Opens video device
Catcher::Catcher(const int &nr) :
        _mut(new boost::mutex), _cam(Camera(nr, &_fr, _mut)), _thr(
                new boost::thread(boost::ref(_cam))) {
}

/// Opens video stream
Catcher::Catcher(const std::string &name) :
        _mut(new boost::mutex), _cam(Camera(name, &_fr, _mut)), _thr(
                new boost::thread(boost::ref(_cam))) {
}

/// Method initializing the object for drawing video stream from a device 
/// with given number
/// \param nr - number of device from which the video stream will be drawn
void Catcher::init(const int &nr) {
    if (_thr != NULL) {
        _thr->interrupt();
        _thr->join();
        delete _thr;
        _thr = NULL;
    }
    if (_mut != NULL) {
        delete _mut;
    }
    _mut = new boost::mutex;
    _cam = Camera(nr, &_fr, _mut);
    _thr = new boost::thread(boost::ref(_cam));
}

/// Method initializing the object for drawing video stream from a video
/// file
/// \param name - name of file from which the video stream will be read
void Catcher::init(const std::string& name) {
    if (_thr != NULL) {
        _thr->interrupt();
        _thr->join();
        delete _thr;
        _thr = NULL;
    }
    if (_mut != NULL) {
        delete _mut;
    }
    _mut = new boost::mutex;
    _cam = Camera(name, &_fr, _mut);
    _thr = new boost::thread(boost::ref(_cam));
}

/// Destructor, ensures that all threads and dynamic objects will be
/// disposed properly
Catcher::~Catcher() {
    if (_thr) {
        _thr->interrupt();
        _thr->join();
        delete _thr;
    }
    if (_mut) {
        delete _mut;
    }
}

/// Method which makes a deep copy of most recent frame, and returns it
/// outside
/// \param frame - object in which the deep copy of most recent frame will
/// be placed
void Catcher::catchFrame(cv::Mat& frame) {
    _mut->lock();
    frame = _fr.clone();
    _mut->unlock();
}

///Method for acquiring cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::get documentation
double Catcher::get(const int &propId) {
    double result;
    _mut->lock();
    result = _cam.get(propId);
    _mut->unlock();
    return result;
}
///Method for setting cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::set documentation
///\param value - value to which property @propId will be set
bool Catcher::set(const int &propId, const double &value) {
    bool result;
    _mut->lock();
    result = _cam.set(propId, value);
    _mut->unlock();
    return result;
}
