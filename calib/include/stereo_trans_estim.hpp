#ifndef STEREO_TRANS_ESTIM
#define STEREO_TRANS_ESTIM

#include <opencv2/core.hpp>
#include <deque>

class StereoTransEstim
{
public:
    StereoTransEstim(void);
    int Calibrate(const std::string& configFile, const std::string& outFile = "");

public:
    int ReadConfig(const std::string& configFile);
    int WriteConfigDefault(const std::string& configFile);
    int BuildCheckerMap(void);
    int CalibrateStereo(void);
    int WriteTrans(const std::string& outFile);

private:
    cv::Size board_size;
    float square_length;
    std::deque< std::string > image_list;
    cv::Size image_size;
    std::vector< std::vector< cv::Point2f> > image_points[2];
    std::vector< std::vector< cv::Point3f> > object_points;
    std::deque<std::string> good_image_list;
    cv::Mat camera_matrix[2];
    cv::Mat dist_coeffs[2];
    cv::Mat rotate_l2r, translate_l2r;
    bool display_corners;
};

#endif
