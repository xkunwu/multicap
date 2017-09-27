#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <string>
#include <deque>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <boost/regex.hpp>

using namespace cv;

int main(int argc, char** argv)
{
    const std::string configFile = "../data/test-trans.xml";

    cv::Size board_size;
    cv::Mat cammatColor, cammatDepth;
    std::deque<std::string> rgbList;
    std::deque<std::string> depList;
    {
        cv::FileStorage storage(configFile, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            std::cout << "test-color2depth: [" << configFile << "] not openned!\n";
            return -1;
        }
        int boardWidth, boardHeight;
        storage["board_width"] >> boardWidth;
        storage["board_height"] >> boardHeight;
        board_size = cv::Size(boardWidth, boardHeight);
        storage["cammatColor0"] >> cammatColor;
        storage["cammatDepth0"] >> cammatDepth;
        {
            cv::FileNode rgbListNode = storage["rgbFileList"];
            if( rgbListNode.type() != cv::FileNode::SEQ ) return -2;
            cv::FileNodeIterator it = rgbListNode.begin(), it_end = rgbListNode.end();
            for( ; it != it_end; ++it ) rgbList.push_back((std::string)*it);
        }
        {
            cv::FileNode depListNode = storage["depFileList"];
            if( depListNode.type() != cv::FileNode::SEQ ) return -1;
            cv::FileNodeIterator it = depListNode.begin(), it_end = depListNode.end();
            for( ; it != it_end; ++it ) depList.push_back((std::string)*it);
        }
        storage.release();
    }

    const int maxScale = 2;
    const float fx = cammatDepth.at<float>(0, 0) / cammatColor.at<float>(0, 0);
    const float fy = cammatDepth.at<float>(1, 1) / cammatColor.at<float>(1, 1);
    const float cx = cammatColor.at<float>(0, 2), cy = cammatColor.at<float>(1, 2);
    const float dx = cammatDepth.at<float>(0, 2), dy = cammatDepth.at<float>(1, 2);
    // const float cx = 960.5, cy = 540.5;
    // const float dx = 256.5, dy = 212.5;
    const size_t num_pair = depList.size();
    if (rgbList.size() != num_pair)
    {
        std::cout << "test-color2depth: different color-depth number!\n";
        return -1;
    }
    for (size_t di = 0; di < num_pair; ++di)
    {
        cv::Mat clr, dep;
        {
            const std::string& filename = rgbList[di];
            clr = cv::imread(filename, 0);
            if(clr.empty())
            {
                std::cout << "test-color2depth: color [" << filename << "] cannot be read!\n";
                break;
            }
        }
        {
            const std::string& filename = depList[di];
            dep = cv::imread(filename, 0);
            if(dep.empty())
            {
                std::cout << "test-color2depth: depth [" << filename << "] cannot be read!\n";
                break;
            }
        }

        bool found = false;
        std::vector<cv::Point2f> corners;
        for( int scale = 1; scale <= maxScale; scale++ )
        {
            cv::Mat timg;
            if( scale == 1 ) timg = clr;
            else cv::resize(clr, timg, cv::Size(), scale, scale);
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
        if (!found)
        {
            std::cout << "test-color2depth: corners not found [" << di << "]!\n";
        }

        const float c0x = 960.5, c0y = 540.5, d0x = 256.5, d0y = 212.5;
        const float masq = 0.2;
        const float mcx = c0x * masq, mcy = c0y * masq, mdx = d0x * masq, mdy = d0y * masq;
        std::vector<cv::Point2f> cornersDep(corners);
        for (size_t cc = 0; cc < cornersDep.size(); ++cc)
        {
            cornersDep[cc].x = (cornersDep[cc].x - cx) * fx + dx;
            cornersDep[cc].y = (cornersDep[cc].y - cy) * fy + dy;
        }
        {
            cv::Mat cimg, cimg1, cimgr;
            {
                cv::cvtColor(clr, cimg, cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners(cimg, board_size, corners, found);
                cv::line(cimg, cv::Point(0, cy), cv::Point(cimg.cols, cy), Scalar(255, 0, 0), 4, 8);
                cv::line(cimg, cv::Point(cx, 0), cv::Point(cx, cimg.rows), Scalar(255, 0, 0), 4, 8);
                cv::line(cimg, cv::Point(0, c0y), cv::Point(cimg.cols, c0y), Scalar(0, 255, 0), 4, 8);
                cv::line(cimg, cv::Point(c0x, 0), cv::Point(c0x, cimg.rows), Scalar(0, 255, 0), 4, 8);
                cv::line(cimg, cv::Point(c0x-mcx, c0y-mcy), cv::Point(c0x-mcx, c0y+mcy), Scalar(0, 255, 0), 4, 8);
                cv::line(cimg, cv::Point(c0x-mcx, c0y+mcy), cv::Point(c0x+mcx, c0y+mcy), Scalar(0, 255, 0), 4, 8);
                cv::line(cimg, cv::Point(c0x+mcx, c0y+mcy), cv::Point(c0x+mcx, c0y-mcy), Scalar(0, 255, 0), 4, 8);
                cv::line(cimg, cv::Point(c0x+mcx, c0y-mcy), cv::Point(c0x-mcx, c0y-mcy), Scalar(0, 255, 0), 4, 8);
                double sf = 640./MIN(clr.rows, clr.cols);
                cv::resize(cimg, cimg1, cv::Size(), sf, sf);
            }
            {
                cv::cvtColor(dep, cimg, cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners(cimg, board_size, cornersDep, found);
                cv::line(cimg, cv::Point(0, dy), cv::Point(cimg.cols, dy), Scalar(255, 0, 0), 1, 8);
                cv::line(cimg, cv::Point(dx, 0), cv::Point(dx, cimg.rows), Scalar(255, 0, 0), 1, 8);
                cv::line(cimg, cv::Point(0, d0y), cv::Point(cimg.cols, d0y), Scalar(0, 255, 0), 1, 8);
                cv::line(cimg, cv::Point(d0x, 0), cv::Point(d0x, cimg.rows), Scalar(0, 255, 0), 1, 8);
                cv::line(cimg, cv::Point(d0x-mdx, d0y-mdy), cv::Point(d0x-mdx, d0y+mdy), Scalar(0, 255, 0), 1, 8);
                cv::line(cimg, cv::Point(d0x-mdx, d0y+mdy), cv::Point(d0x+mdx, d0y+mdy), Scalar(0, 255, 0), 1, 8);
                cv::line(cimg, cv::Point(d0x+mdx, d0y+mdy), cv::Point(d0x+mdx, d0y-mdy), Scalar(0, 255, 0), 1, 8);
                cv::line(cimg, cv::Point(d0x+mdx, d0y-mdy), cv::Point(d0x-mdx, d0y-mdy), Scalar(0, 255, 0), 1, 8);
                double sf = 640./MIN(dep.rows, dep.cols);
                cv::resize(cimg, cimgr, cv::Size(), sf, sf);
            }
            // Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
            //     im1.copyTo(im3(Rect(0, 0, sz1.width, sz1.height)));
            //     im2.copyTo(im3(Rect(sz1.width, 0, sz2.width, sz2.height)));
            cv::imshow("color", cimg1);
            cv::imshow("depth", cimgr);
            char c = (char)cv::waitKey(0);
            if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
            exit(-1);
        }

    }
}
