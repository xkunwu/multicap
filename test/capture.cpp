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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace
{
    libfreenect2::Freenect2 freenect2;
    std::deque<libfreenect2::Freenect2Device *> device_vec;
    // std::deque<libfreenect2::PacketPipeline *> pipeline_vec;
    std::deque<std::string> serial_vec;
    std::deque<libfreenect2::SyncMultiFrameListener *> listener_vec;
    std::deque<libfreenect2::FrameMap> frames_vec;
    std::deque<libfreenect2::Registration*> registration_vec;
    const int depth_y = 424, depth_x = 512;
    libfreenect2::Frame undistorted(depth_x, depth_y, 4), registered(depth_x, depth_y, 4);
    libfreenect2::Frame big_depth(1920, 1082, 4);
}

namespace
{
    bool protonect_shutdown = false; ///< Whether the running application should shut down.
    void sigint_handler(int s)
    {
        protonect_shutdown = true;
    }

    void sigusr1_handler(int s)
    {
        std::cout << s << std::endl;
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
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
    /// [logging]
    #endif
    /// [file logging]
    MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
    if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);
    else
    delete filelogger;
    /// [file logging]

    /// [discovery]
    const int num_devices = freenect2.enumerateDevices();
    if(0 == num_devices)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    frames_vec.resize(num_devices);
    const unsigned frame_type = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    int dev2Show = (1 < argc) ? atoi(argv[1]) : 0;
    if (0 > dev2Show) dev2Show = 0;
    if (num_devices <= dev2Show) dev2Show = 0;

    for (int di = 0; di < num_devices; ++di)
    {
        std::string serial = freenect2.getDeviceSerialNumber(di);
        serial_vec.push_back(serial);
        // libfreenect2::PacketPipeline *pipeline = new libfreenect2::CpuPacketPipeline();
        // //libfreenect2::PacketPipeline *pipeline = new libfreenect2::CudaPacketPipeline();
        // if (nullptr == pipeline)
        // {
        //     std::cout << "cpu pipeline cannot be openned!\n";
        //     return -1;
        // }
        // pipeline_vec.push_back(pipeline);
        // libfreenect2::Freenect2Device* dev = freenect2.openDevice(serial, pipeline);
        libfreenect2::Freenect2Device* dev = freenect2.openDevice(serial);
        device_vec.push_back(dev);
        if(nullptr == dev)
        {
            std::cout << "failure opening device #" << di << " [" << serial << "]!" << std::endl;
            return -1;
        }
        else
        {
            std::cout << "openned device #" << di << " [" << serial << "]" << std::endl;
        }

        libfreenect2::SyncMultiFrameListener* listener = new libfreenect2::SyncMultiFrameListener(frame_type);
        listener_vec.push_back(listener);
        dev->setColorFrameListener(listener);
        dev->setIrAndDepthFrameListener(listener);

        if (!device_vec[di]->start())
        {
            std::cout << "failure starting device #" << di << " [" << serial << "]!" << std::endl;
            return -1;
        }
        std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

        libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
        registration_vec.push_back(registration);
    }
    /// [discovery]

    boost::filesystem::path fullPath( boost::filesystem::initial_path<boost::filesystem::path>() );
    fullPath /= "../data/capture/";
    boost::filesystem::create_directories(fullPath);
    fullPath = boost::filesystem::canonical(fullPath);
    const std::string capPath = fullPath.string();
    // const std::string capPath = boost::filesystem::canonical(fullPath / "capture/").string();
    const std::vector< std::string > typeList = { "rgbFileList", "depFileList", "regFileList", "disFileList", "bigFileList" };
    const size_t num_type = typeList.size();
    std::vector< std::deque<std::string> > fileList(num_type);
    signal(SIGINT,sigint_handler);
    Viewer viewer;
    viewer.initialize();
    size_t framecount = 0;
    size_t framemax = -1;
    const float scaleFactor = 5e3;
    const int depthType = CV_16UC1;
    const int maxDepth = (CV_16UC1 == depthType) ? (1<<16)-1 : (1<<8)-1;
    /// [loop start]
    while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
    {
        for (int di = 0; di < num_devices; ++di)
        {
            if (!listener_vec[di]->waitForNewFrame(frames_vec[di], 10*1000)) // 10 sconds
            {
                std::cout << "device [" << di << "] timeout!" << std::endl;
            }
            // listener_vec[di]->waitForNewFrame(frames_vec[di], 10*1000); // 10 sconds
        }

        for (int di = 0; di < num_devices; ++di)
        {
            libfreenect2::FrameMap& frames = frames_vec[di];
            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
            registration_vec[di]->apply(rgb, depth, &undistorted, &registered, true, &big_depth);
            libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            if (true == viewer.ShouldSave())
            {
                libfreenect2::FrameMap& frames = frames_vec[di];
                {
                    libfreenect2::Frame *frame = frames[libfreenect2::Frame::Color];
                    cv::Mat img(frame->height, frame->width, CV_8UC4, frame->data);
                    cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
                    std::ostringstream oss;
                    oss << capPath << "/camera_" << di << "_rgb_" << framecount << ".png";
                    cv::imwrite(oss.str(), img);
                    fileList[0].push_back(oss.str());
                }
                {
                    libfreenect2::Frame *frame = frames[libfreenect2::Frame::Depth];
                    cv::Mat img(frame->height, frame->width, CV_32FC1, frame->data);
                    img = img / scaleFactor;
                    img.convertTo(img, depthType, maxDepth);
                    std::ostringstream oss;
                    oss << capPath << "/camera_" << di << "_dep_" << framecount << ".png";
                    cv::imwrite(oss.str(), img);
                    fileList[1].push_back(oss.str());
                }
                {
                    cv::Mat img(depth_y, depth_x, CV_8UC4, registered.data);
                    cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
                    std::ostringstream oss;
                    oss << capPath << "/camera_" << di << "_reg_" << framecount << ".png";
                    cv::imwrite(oss.str(), img);
                    fileList[2].push_back(oss.str());
                }
                {
                    cv::Mat img(depth_y, depth_x, CV_32FC1, undistorted.data);
                    img = img / scaleFactor;
                    img.convertTo(img, depthType, maxDepth);
                    std::ostringstream oss;
                    oss << capPath << "/camera_" << di << "_dis_" << framecount << ".png";
                    cv::imwrite(oss.str(), img);
                    fileList[3].push_back(oss.str());
                }
                {
                    cv::Mat img(1082, 1920, CV_32FC1, big_depth.data);
                    img = img / scaleFactor;
                    img.convertTo(img, depthType, maxDepth);
                    std::ostringstream oss;
                    oss << capPath << "/camera_" << di << "_big_" << framecount << ".png";
                    cv::imwrite(oss.str(), img);
                    fileList[4].push_back(oss.str());
                }
                ++ framecount;
            }
            if (dev2Show != di) continue;
            viewer.addFrame("RGB", rgb);
            viewer.addFrame("depth", depth);
            viewer.addFrame("registered", &registered);
            viewer.addFrame("ir", ir);
        }

        for (int di = 0; di < num_devices; ++di)
        {
            protonect_shutdown = protonect_shutdown || viewer.render();
            listener_vec[di]->release(frames_vec[di]);
        }
    }
    /// [loop end]

    const std::string capListFile = capPath + "/capture-list.xml";
    cv::FileStorage storage(capListFile, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        std::cout << "test-capture: [" << capListFile << "] not openned!";
        return -1;
    }
    storage << "scaleFactor" << scaleFactor;
    for (size_t ii = 0; ii < num_type; ++ii)
    {
        storage << typeList[ii] << "[";
        for (size_t fi = 0; fi < framecount; ++fi)
        {
            storage << fileList[ii][fi];
        }
        storage << "]";
    }
    storage.release();

    for (int di = 0; di < num_devices; ++di)
    {
        delete registration_vec[di];
        libfreenect2::Freenect2Device* dev = device_vec[di];
        dev->stop();
        dev->close();
    }

    return 0;
}
