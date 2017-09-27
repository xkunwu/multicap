#include "stereo_trans_estim.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;

StereoTransEstim::StereoTransEstim()
: board_size(cv::Size(9, 6)), square_length(50.f)
{
    display_corners = false;
}

int StereoTransEstim::Calibrate(const std::string& configFile, const std::string& outFile)
{
    ReadConfig(configFile);
    BuildCheckerMap();
    CalibrateStereo();
    WriteTrans(outFile);
}

int StereoTransEstim::WriteConfigDefault(const std::string& configFile)
{
    cv::FileStorage storage(configFile, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        std::cout << "StereoTransEstim::WriteConfigDefault: [" << configFile << "] not openned!";
        return -1;
    }
    //cv::cvWriteComment(*storage, "a double value", 0);
    storage << "board_width" << board_size.width;
    storage << "board_height" << board_size.height;
    storage << "square_length" << square_length;
    storage << "display_corners" << display_corners;
    storage << "cammatColor0" << (
        cv::Mat_<float>(3,3) <<
        1081.3721, 0., 959.5,
        0., 1081.3721, 539.5,
        0., 0., 1.
    );
    storage << "cammatDepth0" << (
        cv::Mat_<float>(3,3) <<
        365.39, 0., 257.374,
        0., 365.39, 203.933,
        0., 0., 1.
    );
    storage.release();
    return 0;
}
int StereoTransEstim::ReadConfig(const std::string& configFile)
{
    cv::FileStorage storage(configFile, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        std::cout << "StereoTransEstim::ReadConfig: [" << configFile << "] not openned!";
        return -1;
    }
    int boardWidth, boardHeight;
    storage["board_width"] >> boardWidth;
    storage["board_height"] >> boardHeight;
    board_size = cv::Size(boardWidth, boardHeight);
    storage["square_length"] >> square_length;
    storage["display_corners"] >> display_corners;
    storage["cammatColor0"] >> camera_matrix[0];
    storage["cammatColor0"] >> camera_matrix[1];
    cv::FileNode imageListNode = storage["rgbFileList"];
    {
        if( imageListNode.type() != cv::FileNode::SEQ ) return -1;
        image_list.clear();
        cv::FileNodeIterator it = imageListNode.begin(), it_end = imageListNode.end();
        for( ; it != it_end; ++it ) image_list.push_back((std::string)*it);
    }
    storage.release();
    return 0;
}

int StereoTransEstim::BuildCheckerMap(void)
{
    if (image_list.size() % 2 != 0)
    {
        std::cout << "StereoTransEstim::BuildCheckerMap: odd (non-even) number of image list\n";
        return -1;
    }

    const int maxScale = 2;
    int i, j, k, nimages = (int)image_list.size()/2;

    image_points[0].resize(nimages);
    image_points[1].resize(nimages);

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const std::string& filename = image_list[i*2+k];
            // std::cout << filename << std::endl;
            cv::Mat img = cv::imread(filename, 0);
            if(img.empty())
            {
                std::cout << "StereoTransEstim::BuildCheckerMap: image [" << filename << "] cannot be read!\n";
                break;
            }
            if( 0 == k ) image_size = img.size();
            else if( img.size() != image_size )
            {
                std::cout << "StereoTransEstim::BuildCheckerMap: image [" << filename << "] has different size from the first\n";
                break;
            }
            bool found = false;
            std::vector<cv::Point2f>& corners = image_points[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                cv::Mat timg;
                if( scale == 1 ) timg = img;
                else cv::resize(img, timg, cv::Size(), scale, scale);
                found = cv::findChessboardCorners(
                    timg, board_size, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
                );
                if (!corners.empty() &&
                    corners.front().x < corners.back().x &&
                    corners.front().y < corners.back().y)
                {
                    std::reverse(corners.begin(), corners.end());
                }
                if( found )
                {
                    if( scale > 1 )
                    {
                        cv::Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( display_corners )
            {
                std::cout << filename << std::endl;
                cv::Mat cimg, cimg1;
                cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners(cimg, board_size, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                cv::resize(cimg, cimg1, cv::Size(), sf, sf);
                cv::imshow("corners", cimg1);
                char c = (char)cv::waitKey(0);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                exit(-1);
            }
            else
            putchar('.');
            if( !found )
            break;
            cv::cornerSubPix(
                img, corners, cv::Size(11,11), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01)
            );
        }
        if( k == 2 )
        {
            good_image_list.push_back(image_list[i*2]);
            good_image_list.push_back(image_list[i*2+1]);
            j++;
        }
    }
    std::cout << "StereoTransEstim::BuildCheckerMap: " << j << " pairs have been successfully detected.\n";
    nimages = j;
    // if( nimages < 2 )
    // {
    //     std::cout << "StereoTransEstim::BuildCheckerMap: too little pairs to run the calibration!\n";
    //     return -1;
    // }

    image_points[0].resize(nimages);
    image_points[1].resize(nimages);
    object_points.resize(nimages);
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < board_size.height; j++ )
        for( k = 0; k < board_size.width; k++ )
        object_points[i].push_back(cv::Point3f(k*square_length, j*square_length, 0));
    }

    return 0;
}

int StereoTransEstim::CalibrateStereo(void)
{
    cv::Mat distCoeffs[2];
    cv::Mat R, T, E, F;

    std::cout << "StereoTransEstim::CalibrateStereo: running stereo calibration ...\n";
    double rms = stereoCalibrate(
        object_points, image_points[0], image_points[1],
        camera_matrix[0], distCoeffs[0],
        camera_matrix[1], distCoeffs[1],
        image_size, R, T, E, F,
        cv::CALIB_FIX_ASPECT_RATIO +
        cv::CALIB_ZERO_TANGENT_DIST +
        cv::CALIB_FIX_INTRINSIC +
        cv::CALIB_SAME_FOCAL_LENGTH +
        cv::CALIB_RATIONAL_MODEL +
        cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5)
    );
    R.convertTo(rotate_l2r, CV_32F);
    T.convertTo(translate_l2r, CV_32F);
    std::cout << "StereoTransEstim::CalibrateStereo: done with RMS error=" << rms << std::endl;


    // double err = 0;
    // int npoints = 0;
    // vector<Vec3f> lines[2];
    // for( i = 0; i < nimages; i++ )
    // {
    //     int npt = (int)image_points[0][i].size();
    //     Mat imgpt[2];
    //     for( k = 0; k < 2; k++ )
    //     {
    //         imgpt[k] = Mat(image_points[k][i]);
    //         undistortPoints(imgpt[k], imgpt[k], camera_matrix[k], distCoeffs[k], Mat(), camera_matrix[k]);
    //         computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
    //     }
    //     for( j = 0; j < npt; j++ )
    //     {
    //         double errij = fabs(
    //             image_points[0][i][j].x*lines[1][j][0] +
    //             image_points[0][i][j].y*lines[1][j][1] + lines[1][j][2]
    //         ) + fabs(
    //             image_points[1][i][j].x*lines[0][j][0] +
    //             image_points[1][i][j].y*lines[0][j][1] + lines[0][j][2]
    //         );
    //         err += errij;
    //     }
    //     npoints += npt;
    // }
    // cout << "StereoTransEstim::CalibrateStereo: average epipolar err = " <<  err/npoints << endl;
}

int StereoTransEstim::WriteTrans(const std::string& outFile)
{
    if (outFile.empty()) return -1;
    cv::FileStorage storage(outFile, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        std::cout << "StereoTransEstim::WriteTrans: [" << outFile << "] not openned!";
        return -1;
    }
    storage << "R" << rotate_l2r;
    storage << "T" << translate_l2r;
    storage.release();
    return 0;
}
