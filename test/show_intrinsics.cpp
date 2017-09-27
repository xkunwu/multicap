#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include "src/viewer.h"

#include <iostream>
#include <sstream>
#include <deque>
#include <cstdlib>
#include <signal.h>
#include <opencv2/core.hpp>

namespace
{
    bool protonect_shutdown = false; ///< Whether the running application should shut down.
    void sigint_handler(int s)
    {
        protonect_shutdown = true;
    }

    bool protonect_paused = false;
    libfreenect2::Freenect2Device *devtopause = nullptr;

    //Doing non-trivial things in signal handler is bad. If you want to pause,
    //do it in another thread.
    //Though libusb operations are generally thread safe, I cannot guarantee
    //everything above is thread safe when calling start()/stop() while
    //waitForNewFrame().
    void sigusr1_handler(int s)
    {
        if (devtopause == 0)
        return;
        /// [pause]
        if (protonect_paused)
        devtopause->start();
        else
        devtopause->stop();
        protonect_paused = !protonect_paused;
        /// [pause]
    }
}

#include <fstream>
#include <cstdlib>
class MyFileLogger: public libfreenect2::Logger
{
private:
    std::ofstream logfile_;
public:
    MyFileLogger(const char *filename)
    {
        if (filename)
        logfile_.open(filename);
        level_ = Debug;
    }
    bool good()
    {
        return logfile_.is_open() && logfile_.good();
    }
    virtual void log(Level level, const std::string &message)
    {
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
    }
};

/// [main]
/**
* Main application entry point.
*
* Accepted argumemnts:
* - cpu Perform depth processing with the CPU.
* - gl  Perform depth processing with OpenGL.
* - cl  Perform depth processing with OpenCL.
* - <number> Serial number of the device to open.
* - -noviewer Disable viewer window.
*/
int main(int argc, char *argv[])
/// [main]
{
    std::string program_path(argv[0]);
    std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;
    std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;

    size_t executable_name_idx = program_path.rfind("test");
    std::string binpath = "/";
    if(executable_name_idx != std::string::npos)
    {
        binpath = program_path.substr(0, executable_name_idx);
    }

    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    // avoid flooing the very slow Windows console with debug messages
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
    #else
    // create a console logger with debug level (default is console logger with info level)
    /// [logging]
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
    /// [logging]
    #endif
    /// [file logging]
    MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
    if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);
    else
    delete filelogger;
    /// [file logging]

    /// [context]
    libfreenect2::Freenect2 freenect2;
    std::deque<libfreenect2::Freenect2Device *> device_vec;
    std::deque<libfreenect2::PacketPipeline *> pipeline_vec;
    /// [context]
    std::deque<std::string> serial_vec;

    bool viewer_enabled = true;
    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;
    size_t framemax = -1;

    /// [discovery]
    const int num_devices = freenect2.enumerateDevices();
    if(0 == num_devices)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string filename = "../data/test-cammat.xml";
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        std::cout << "show_intrinsics: [" << filename << "] not openned!";
        return -1;
    }
    for (int di = 0; di < num_devices; ++di)
    {
        std::string serial = freenect2.getDeviceSerialNumber(di);
        serial_vec.push_back(serial);
        libfreenect2::PacketPipeline *pipeline = new libfreenect2::CpuPacketPipeline();
        //libfreenect2::PacketPipeline *pipeline = new libfreenect2::CudaPacketPipeline();
        if (nullptr == pipeline)
        {
            std::cout << "cpu pipeline cannot be openned!\n";
            return -1;
        }
        pipeline_vec.push_back(pipeline);
        libfreenect2::Freenect2Device* dev = freenect2.openDevice(serial, pipeline);
        device_vec.push_back(dev);
        if(nullptr == dev)
        {
            std::cout << "failure opening device #" << di << " [" << serial << "]!" << std::endl;
            return -1;
        }
        else
        {
            std::cout << "openned device #" << di << " [" << serial << "]!" << std::endl;
        }
        dev->start();
        const libfreenect2::Freenect2Device::ColorCameraParams param_color = dev->getColorCameraParams();
        {
            std::cout << "color camera parameter: "
            << param_color.fx << " "
            << param_color.fy << " "
            << param_color.cx << " "
            << param_color.cy << std::endl;
            cv::Mat pacol_mat = cv::Mat::eye(3, 3, CV_32F);
            pacol_mat.at<float>(0, 0) = param_color.fx;
            pacol_mat.at<float>(1, 1) = param_color.fy;
            pacol_mat.at<float>(0, 2) = param_color.cx;
            pacol_mat.at<float>(1, 2) = param_color.cy;
            std::cout << pacol_mat << std::endl;
            std::ostringstream oss;
            oss << "cammatColor" << di;
            if (storage.isOpened()) storage << oss.str() << pacol_mat;
        }
        const libfreenect2::Freenect2Device::IrCameraParams param_depth = dev->getIrCameraParams();
        {
            std::cout << "depth camera parameter: "
            << param_depth.fx << " "
            << param_depth.fy << " "
            << param_depth.cx << " "
            << param_depth.cy << std::endl;
            cv::Mat padep_mat = cv::Mat::eye(3, 3, CV_32F);
            padep_mat.at<float>(0, 0) = param_depth.fx;
            padep_mat.at<float>(1, 1) = param_depth.fy;
            padep_mat.at<float>(0, 2) = param_depth.cx;
            padep_mat.at<float>(1, 2) = param_depth.cy;
            std::cout << padep_mat << std::endl;
            std::ostringstream oss;
            oss << "cammatDepth" << di;
            if (storage.isOpened()) storage << oss.str() << padep_mat;
        }
    }
    storage.release();
    /// [discovery]

    for (int di = 0; di < num_devices; ++di)
    {
        libfreenect2::Freenect2Device* dev = device_vec[di];
        dev->stop();
        dev->close();
    }

    return 0;
}
