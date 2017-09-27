#include "opencv2/core.hpp"
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

#include<libfreenect2/registration.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/ply_io.h>

#include "c2d_svd_trans_estim.hpp"

using namespace cv;

int main(int argc, char** argv)
{
    std::string configFile = (1 < argc) ? std::string(argv[1]) : "../data/test-trans.xml";
    const std::string transFile = "../data/trans-l2r.xml";
    const std::string outPlyFile = "../data/trans.ply";

    cv::Mat cammatColor, cammatDepth;
    std::deque<std::string> rgbList, depList;
    float scaleFactor;
    {
        cv::FileStorage storage(configFile, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            std::cout << "test-trans: [" << configFile << "] not openned!\n";
            return -1;
        }
        storage["cammatColor0"] >> cammatColor;
        storage["cammatDepth0"] >> cammatDepth;
        storage["scaleFactor"] >> scaleFactor;
        cv::FileNode rgbListNode = storage["rgbFileList"];
        {
            if( rgbListNode.type() != cv::FileNode::SEQ ) return -1;
            cv::FileNodeIterator it = rgbListNode.begin(), it_end = rgbListNode.end();
            for( ; it != it_end; ++it ) rgbList.push_back((std::string)*it);
        }
        cv::FileNode depListNode = storage["depFileList"];
        {
            if( depListNode.type() != cv::FileNode::SEQ ) return -1;
            cv::FileNodeIterator it = depListNode.begin(), it_end = depListNode.end();
            for( ; it != it_end; ++it ) depList.push_back((std::string)*it);
        }
        storage.release();
    }

    cv::Mat rota, tran;
    {
        cv::FileStorage storage(transFile, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            std::cout << "test-trans: [" << transFile << "] not openned!\n";
            return -1;
        }
        storage["R"] >> rota;
        storage["T"] >> tran;
        storage.release();
        // std::cout << rota << std::endl;
        // std::cout << tran << std::endl;
        cv::transpose(rota, rota);
    }

    C2DSVDTransEstim::CamProj camProj(cammatDepth);
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    const size_t numDepth = depList.size();
    for (size_t di = 0; di < 2; ++di)
    {
        const std::string& depFile = depList[di];
        cv::Mat dep = cv::imread(depFile, cv::IMREAD_ANYDEPTH);
        if(dep.empty())
        {
            std::cout << "test-trans: depth [" << depFile << "] cannot be read!\n";
            break;
        }
        const int depthType = dep.depth();
        int maxDepth;
        // const int maxDepth = (CV_16UC1 == depthType) ? (1<<16)-1 : (1<<8)-1;
        if (CV_8U == depthType) maxDepth = (1<<8)-1;
        else if (CV_16U == depthType) maxDepth = (1<<16)-1;
        else
        {
            std::cout << "C2DSVDTransEstim::BuildCheckerMap: unsupported depth type!\n";
            return -1;
        }
        int camid = 0;
        {
            boost::smatch match;
            boost::regex_search(depFile, match, boost::regex("camera.*(\\d)"));
            // std::cout << match[1] << std::endl;
            camid = boost::lexical_cast<int>(match[1]);
            // std::cout << camid << std::endl;
        }
        dep.convertTo(dep, CV_32FC1, scaleFactor/maxDepth);

        const std::string& rgbFile = rgbList[di];
        cv::Mat rgb = cv::imread(rgbFile, cv::IMREAD_COLOR);
        if(rgb.empty())
        {
            std::cout << "test-trans: rgb [" << rgbFile << "] cannot be read!\n";
            break;
        }
        // std::cout << rgbFile << "\n";
        // std::cout << depFile << "\n";

        const int rows = dep.rows, cols = dep.cols;
        for (int ri = 0; ri < rows; ++ri)
        {
            for (int ci = 0; ci < cols; ++ci)
            {
                const float dij = dep.at<float>(ri, ci);
                if (
                    std::isnan(dij)
                    // || 5e2 > dij || 2.5e3 < dij
                ) continue;
                cv::Mat pos = cv::Mat(camProj.Get3D(ri, ci) * dij);
                // if (0 == di)
                // {
                //     pos = rota * pos + tran;
                // }
                if (0 < di)
                {
                    pos = rota * (pos - tran);
                }
                pos = pos / 1e3; // scale to meter
                pcl::PointXYZRGB point;
                point.x = pos.at<float>(0);
                point.y = pos.at<float>(1);
                point.z = pos.at<float>(2);
                // point.rgb = rgb.at<float>(ri, ci);
                point.rgb = *reinterpret_cast<float*>(&rgb.at<cv::Vec3b>(ri, ci));
                pointCloud.push_back(point);
            }
        }
        std::putchar('.');
    }
    std::putchar('\n');
    pcl::io::savePLYFile(outPlyFile, pointCloud);
    return 0;
}
