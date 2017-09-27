#include "stereo_trans_estim.hpp"
#include "c2d_svd_trans_estim.hpp"

#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;

int main(int argc, char** argv)
{
    // StereoTransEstim stestim;
    // std::string configFile = (1 < argc) ? std::string(argv[1]) : "../data/test-trans.xml";
    // stestim.Calibrate(configFile, "../data/trans-l2r.xml");

    C2DSVDTransEstim svdestim;
    std::string configFile = (1 < argc) ? std::string(argv[1]) : "../data/test-trans.xml";
    svdestim.Calibrate(configFile, "../data/trans-l2r.xml");

    return 0;
}
