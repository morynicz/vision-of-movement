/*
 * \file SerialPortTest.cpp
 *
 * \date Jun 7, 2013
 * \author Michał Orynicz
 */
//#include "SerialPort.hpp"
#include <cstdlib>
#include <string>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <utility>
#include <list>
#include <iostream>
#include <sstream>
#include <fstream>

const int FAIL = -1;
const int SUCCESS = 0;

/// Conversion from number to termios baudrate symbol
int getTermiosBaudRate(const int &baud);

/// Check if termios structs are the same
std::string compareTermiosStruct(const struct termios &original,
        const struct termios &check);

/// Class encapsulating a measurement from the test vehicle
class Measurement {
    public:
        long procTime; ///<time stamp from vehicle processor [0.01 s]
        int alpha; ///<front wheels turn angle
        int velocity; ///<linear velocity
        long x; ///<x coordinate [mm]
        long y; ///<y coordinate [mm]
        long theta; ///<orientation [0.01 deg]
        long long stamp; ///<recieving system timestamp [ms since epoch]

        ///Default constructor
        Measurement() :
                procTime(0), alpha(0), velocity(0), x(0), y(0), theta(
                        0), stamp(0) {

        }

        /// Full initialisation constructor
        /**
         * Full initialisation constructor. Allows to assign values
         * to all fields in one instruction.
         * @param procTime - on-board processor timestamp [0.01s]
         * @param alpha - front wheels turn angle
         * @param velocity - linear velocity of vehicle
         * @param x - x coordinate [mm]
         * @param y - y coordinate [mm]
         * @param theta- orientation [0.1 deg]
         * @param timestamp - timestamp for when the measurement
         * was received [ms since unix epoch]
         */
        Measurement(const long &procTime_, const int &alpha_,
                const int &velocity_, const long &x_, const long &y_,
                const long &theta_, const long long &stamp_) :
                procTime(procTime_), alpha(alpha_), velocity(
                        velocity_), x(x_), y(y_), theta(theta_), stamp(
                        stamp_) {
        }

        ///Function decoding a c-string to a measurement
        /**
         * Function decodes string str to a measurement with the given
         * timestamp
         * @param str - string to be decoded
         * @param timestamp - miliseconsd since unix epoch
         *
         * @retVal number of parameters read
         */
        int readMeasurement(const char *str, long long timeStamp) {
            stamp = timeStamp;
            return sscanf(str, "%ld %d %d %ld %ld %ld \n", &procTime,
                    &alpha, &velocity, &x, &y, &theta);

        }
};

std::ostream &operator<<(std::ostream &stream,
        const Measurement &measure) {
    stream << measure.procTime << " " << measure.alpha << " "
            << measure.velocity << " " << measure.x << " "
            << measure.y << " " << measure.theta << " "
            << measure.stamp;
    return stream;
}

/// Class modelling serial port
/**
 * Class providing abstraction for reading and writing from
 * serial port
 */
class PortReader {
        int _fd; ///< port file descriptor
        std::list<Measurement> _buffer; ///< Measurements buffer
        std::mutex mutex; ///< mutex for _buffer access
        bool _stop; ///< flag telling when to stop measurements
        std::ofstream _output; ///<file to save the measurements read
    public:
        /// Basic constructor initializing port descriptor and output file
        PortReader(const int &fd, const std::string &filename) :
                _fd(fd), _stop(false) {
            _output.open(filename.c_str(), std::ofstream::out);
            if (!_output.is_open()) { //FIXME: this is not done well
                throw "could not open file " + filename;
            }
        }
        /// Functor method for launching thread
        /**
         * Functor method for launching a separate reading thread.
         * Reads messages from port in infinte loop. Processes message
         * after reading \n, save it to buffer and write to file.
         * Can be stopped using stop().
         */
        void operator()() {
            int buffSize = 1;
            char ch;
            std::stringstream strBuff;
            char buffer[255];
            while (!_stop) {
                std::cerr << "op(): ";
                strBuff.flush();
                strBuff.clear();
                do {
                    //      ioMut.lock();
//                    std::cerr << "plum" << std::endl;
                    read(_fd, &ch, buffSize);
//                    std::cerr << "plom" << std::endl;
                    //    ioMut.unlock();
                    std::cerr << ch;
                    strBuff << ch;
                } while ('\n' != ch && '\r' != ch);
                std::cerr << "complete" << std::endl;
                //get the timestamp
                std::chrono::high_resolution_clock::time_point now =
                        std::chrono::high_resolution_clock::now();
                Measurement measurement;
                std::chrono::high_resolution_clock::duration duration =
                        now.time_since_epoch();
                long long miliseconds = std::chrono::duration_cast<
                        std::chrono::milliseconds>(duration).count();
                strBuff.getline(buffer, 255);
                strBuff.flush();
                std::cerr << "op():buffer: " << buffer << std::endl;
                measurement.readMeasurement(buffer, miliseconds);
                std::cerr << "op():measurement read: " << measurement
                        << std::endl;
                _output << measurement << std::endl;
                mutex.lock();
                _buffer.push_back(measurement);
                mutex.unlock();
            }

        }

        /**
         * Get measurements from buffer emptying it
         * @param history - list of measurements to which
         * the ones from the buffer will be appended
         */
        void getMeasurments(std::list<Measurement> &history) {
            mutex.lock();
            history.insert(history.begin(), _buffer.begin(),
                    _buffer.end());
            _buffer.clear();
            mutex.unlock();
        }

        /// Stop the inifinite loop in operator()
        void stop() {
            _stop = true;
        }

        ///Send the string given through the port
        /**
         * Send the given string through the port
         * @param comm - string to be sent
         */
        int send(const std::string & comm) {
            std::cerr << strlen(comm.c_str()) << " " << comm.size()
                    << std::endl;

            int result = write(_fd, comm.c_str(), comm.size());
            return result;
        }

        ///Send the string given through the port
        /**
         * Send the given string through the port
         * @param comm - string to be sent
         */

        int send(const char* comm) {
            std::cerr << "send: " << comm << std::endl;
            int toWrite = strlen(comm);
            std::cerr << "send: toWrite: " << toWrite << std::endl;
            std::cerr << "send: in hex" << std::endl;
            for (int i = 0; i < toWrite; ++i) {
                fprintf(stderr, "%X ", comm[i]);
            }
            std::cerr << std::endl;
            while (0 < toWrite) {
                int written = write(_fd, comm, 1);
                toWrite -= written;
                std::cerr << "send: written: " << written
                        << std::endl;
                std::cerr<<"sent:"<<*comm<<std::endl;;
                comm += written;

                usleep(100000);
            }
            std::cerr << "send: write complete" << std::endl;
            return 0;
        }

        ~PortReader() {
            _output.close();
        }

};

int main(int argc, char** argv) {

    if (4 != argc) {
        std::cerr << "Error: bad number of arguments. Should be:"
                << "\n " << argv[0]
                << " <baud rate> <port> <output_file>" << std::endl;
        return FAIL;
    }

    struct termios options;
    // Read and write access
    // Don't make the terminal opened the controlling terminal for the process
    // Open in non-blocking mode
    int portId = open(argv[2], O_RDWR | O_NOCTTY | O_NDELAY);
    if (portId == -1) {
        std::cerr
                << "Error: could not open file "
                        + std::string(argv[2]);
        return FAIL;
    } else {
        fcntl(portId, F_SETFL, 0); //set all file description bits to 0

        tcgetattr(portId, &options); // get current settings
        int baudRate = getTermiosBaudRate(atoi(argv[1]));
        if (0 < baudRate) {
            cfsetspeed(&options, baudRate); // set baud rate

            options.c_cflag &= ~CSIZE; // Mask the character size bits
            options.c_cflag |= CS8; // 8 bit data
            options.c_cflag &= ~PARENB; // set parity to no
            options.c_cflag &= ~PARODD; // set parity to no
            options.c_cflag &= ~CSTOPB; //set one stop bit

            //enabale receiver
            options.c_cflag |=  CREAD;

            //enable implementation defined output processing
            options.c_oflag &= ~OPOST;

            options.c_lflag = 0;
            options.c_iflag = 0; //disable software flow controll
            options.c_oflag = 0;
            options.c_cflag |= CRTSCTS; //enable hardware flow controll

            options.c_iflag = IGNPAR | IGNBRK;

            //  cfmakeraw(&options); //
            options.c_cc[VTIME] = 0;
            options.c_cc[VMIN] = 1; //switch to raw mode (no i/o processing)

            if (tcsetattr(portId, TCSANOW, &options)) { // save the settings
                struct termios check;
                tcgetattr(portId, &check);
                std::string message = compareTermiosStruct(options,
                        check);
                if (!message.empty()) {
                    std::cerr << message << std::endl;
                    return FAIL;
                }

            }

            tcflush(portId, TCOFLUSH);
            tcflush(portId, TCIFLUSH);

            std::list<Measurement> msgs; //the received measurements

            PortReader reader(portId, argv[3]);

            std::thread thread(std::ref(reader)); //launch the thread
            msgs.push_front(Measurement(0, 0, 0, 0, 0, 0, 0));
            std::string in;
            std::stringstream buff;
            char strr[255];
            do {
                std::cin.getline(strr, 255);
                strr[strlen(strr) + 1] = '\0';
                strr[strlen(strr)] = 0x0d;
                std::cerr << strr << std::endl;
                if (NULL == strchr(strr, 'q')) {
                    reader.send(strr);
                }
            } while (NULL == strchr(strr, 'q'));
            reader.stop(); //stop the reading thread
            reader.getMeasurments(msgs);
            thread.join();
            for (std::list<Measurement>::iterator it = msgs.begin();
                    msgs.end() != it; ++it) {
                std::cerr << "proc: " << it->procTime << " alpha: "
                        << it->alpha << " velocity: " << it->velocity
                        << " x: " << it->x << " y: " << it->y
                        << " theta: " << it->theta << " stamp: "
                        << it->stamp << std::endl;
            }

        }
    }
    return SUCCESS;
}

/**
 * Convert baudrate value totermios symbol
 * @parm baud - baudrate value to be converted
 * @retVal - termios symbol for the baudrate
 */
int getTermiosBaudRate(const int &baud) {
    int result = -1;
    switch (baud) {
        case 0:
            result = B0;
            break;
        case 50:
            result = B50;
            break;
        case 75:
            result = B75;
            break;
        case 110:
            result = B110;
            break;
        case 134:
            result = B134;
            break;
        case 150:
            result = B150;
            break;
        case 200:
            result = B200;
            break;
        case 300:
            result = B300;
            break;
        case 600:
            result = B600;
            break;
        case 1200:
            result = B1200;
            break;
        case 1800:
            result = B1800;
            break;
        case 2400:
            result = B2400;
            break;
        case 4800:
            result = B4800;
            break;
        case 9600:
            result = B9600;
            break;
        case 19200:
            result = B19200;
            break;
        case 38400:
            result = B38400;
            break;
        case 57600:
            result = B57600;
            break;
        case 115200:
            result = B115200;
            break;
        case 230400:
            result = B230400;
            break;
        default:
            result = -1;
            break;
    }
    return result;
}

std::string compareTermiosStruct(const struct termios &original,
        const struct termios &check) {
    std::string message = "";
    if (original.c_cflag != check.c_cflag) {
        std::cerr << "c_cflag mismatch" << std::endl;
        message += "c_cflag mismatch";
    }
    if (original.c_iflag != check.c_iflag) {
        std::cerr << "c_iflag mismatch" << std::endl;
        message += "c_iflag mismatch";
    }
    if (original.c_lflag != check.c_lflag) {
        std::cerr << "c_lflag mismatch" << std::endl;
        message += "c_lflag mismatch";
    }
    if (original.c_oflag != check.c_oflag) {
        std::cerr << "c_oflag mismatch" << std::endl;
        message += "c_oflag mismatch";
    }
    if (original.c_ispeed != check.c_ispeed) {
        std::cerr << "c_ispeed mismatch" << std::endl;
        message += "c_ispeed mismatch";
    }
    if (original.c_ospeed != check.c_ospeed) {
        std::cerr << "c_ospeed mismatch" << std::endl;
        message += "c_ospeed mismatch";
    }
    if (original.c_line != check.c_line) {
        std::cerr << "c_line mismatch" << std::endl;
        message += "c_line mismatch";
    }
    return message;
}
