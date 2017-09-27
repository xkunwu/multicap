#include "c2d_svd_trans_estim.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <string>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/ply_io.h>

using namespace cv;

int C2DSVDTransEstim::ComputeTransSVD(void)
{
    // pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    const float sf = 1e5;
    cv::Matx33f ident = cv::Matx33f::eye();
    cv::Matx33f cov = cv::Matx33f::zeros();
    for (size_t nf = 0; nf < frames.size(); ++nf)
    {
        const size_t npairs = frames[nf].szPoints;
        for (size_t pp = 0; pp < npairs; ++pp)
        {
            const cv::Point3f pl = frames[nf].depPoints[0][pp];
            const cv::Point3f pr = frames[nf].depPoints[1][pp];
            if (1 > pl.z || 1 > pr.z) continue;
            const cv::Vec3f vl = (pl - frames[nf].centroid[0]) / sf;
            const cv::Vec3f vr = (pr - frames[nf].centroid[1]) / sf;
            for (int ii = 0; ii < 3; ++ii)
            for (int jj = 0; jj < 3; ++jj)
            cov(ii, jj) += vl(ii) * vr(jj);

            // {
            //     pcl::PointXYZRGB point(0, 0, 255);
            //     point.x = pl.x;
            //     point.y = pl.y;
            //     point.z = pl.z;
            //     pointCloud.push_back(point);
            // }
            // {
            //     pcl::PointXYZRGB point(255, 0, 0);
            //     point.x = pr.x;
            //     point.y = pr.y;
            //     point.z = pr.z;
            //     pointCloud.push_back(point);
            // }
        }
        // {
        //     pcl::PointXYZRGB point(0, 255, 255);
        //     point.x = frames[nf].centroid[0].x;
        //     point.y = frames[nf].centroid[0].y;
        //     point.z = frames[nf].centroid[0].z;
        //     pointCloud.push_back(point);
        // }
        // {
        //     pcl::PointXYZRGB point(255, 255, 0);
        //     point.x = frames[nf].centroid[1].x;
        //     point.y = frames[nf].centroid[1].y;
        //     point.z = frames[nf].centroid[1].z;
        //     pointCloud.push_back(point);
        // }
    }
    // pcl::io::savePLYFile("../data/test-cov.ply", pointCloud);
    // std::cout << "cov: \n" << cov << std::endl;
    cv::Matx33f u, vt;
    cv::Matx31f w;
    cv::SVD::compute(cov, w, u, vt);
    // std::cout << w << std::endl;
    // std::cout << u << std::endl;
    // std::cout << vt << std::endl;
    cv::Matx33f ut = u.t(), v = vt.t();
    double det = cv::determinant(v * ut);
    if (0 > det) ident(2, 2) = -1;
    rotate_l2r = v * ident * ut;
    // rotate_l2r = cv::Mat(cv::Matx33f::eye());
    translate_l2r = centroid[1] - rotate_l2r * centroid[0];
    std::cout << "rotate: \n" << rotate_l2r << std::endl;
    std::cout << "translate: \n" << translate_l2r << std::endl;
    return 0;
}

C2DSVDTransEstim::C2DSVDTransEstim()
: board_size(cv::Size(9, 6))
{
    display_corners = true;
    depth_bits = -1;
}

int C2DSVDTransEstim::Calibrate(const std::string& configFile, const std::string& outFile)
{
    ReadConfig(configFile);
    BuildCheckerMap();
    ComputeTransSVD();
    TestTrans("../data/trans.ply");
    WriteTrans(outFile);
}

int C2DSVDTransEstim::ReadConfig(const std::string& configFile)
{
    cv::FileStorage storage(configFile, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        std::cout << "C2DSVDTransEstim::ReadConfig: [" << configFile << "] not openned!";
        return -1;
    }
    int boardWidth, boardHeight;
    storage["board_width"] >> boardWidth;
    storage["board_height"] >> boardHeight;
    board_size = cv::Size(boardWidth, boardHeight);
    storage["scaleFactor"] >> scaleFactor;
    storage["cammatDepth0"] >> camera_matrix;
    camProj = CamProj(camera_matrix);
    storage["display_corners"] >> display_corners;
    cv::FileNode imageListNode = storage["rgbFileList"];
    {
        if( imageListNode.type() != cv::FileNode::SEQ ) return -1;
        image_list.clear();
        cv::FileNodeIterator it = imageListNode.begin(), it_end = imageListNode.end();
        for( ; it != it_end; ++it ) image_list.push_back((std::string)*it);
    }
    cv::FileNode depListNode = storage["depFileList"];
    {
        if( depListNode.type() != cv::FileNode::SEQ ) return -1;
        depth_list.clear();
        cv::FileNodeIterator it = depListNode.begin(), it_end = depListNode.end();
        for( ; it != it_end; ++it ) depth_list.push_back((std::string)*it);
    }
    storage.release();
    return 0;
}

C2DSVDTransEstim::CamProj::CamProj(void)
{
    fx = fy = 1.f;
    cx = cy = 0.f;
}
C2DSVDTransEstim::CamProj::CamProj(const cv::Mat& cammat)
{
    fx = cammat.at<float>(0, 0);
    fy = cammat.at<float>(1, 1);
    cx = cammat.at<float>(0, 2);
    cy = cammat.at<float>(1, 2);
}
cv::Point3f C2DSVDTransEstim::CamProj::Get3D(const int& r, const int& c) const
{
    return cv::Point3f(
        ((float)c + 0.5 - cx) / fx,
        - ((float)r + 0.5 - cy) / fy,
        1
    );
}

void C2DSVDTransEstim::FramePair::DrawCorners(void)
{
    cv::Mat rgbdep[2];
    const int ws = 512;
    for (int k = 0; k < 2; ++k)
    {
        cv::Mat cimg, dimg;
        cimg = rgb[k].clone();
        // cv::cvtColor(rgb[k], cimg, cv::COLOR_GRAY2BGR);
        if (CV_16UC1 == dep[k].depth()) dep[k].convertTo(dimg, CV_8UC1, 1.f/255);
        else dimg = dep[k].clone();
        cv::cvtColor(dimg, dimg, cv::COLOR_GRAY2BGR);
        if (!rgbPoints[k].empty())
        {
            cv::drawChessboardCorners(cimg, szBoard, rgbPoints[k], true);
            cv::drawChessboardCorners(dimg, szBoard, rgbPoints[k], true);
        }
        double sf = (double)ws/rgb[k].cols;
        cv::resize(cimg, cimg, cv::Size(), sf, sf);
        cv::resize(dimg, dimg, cv::Size(), sf, sf);
        cv::vconcat(cimg, dimg, rgbdep[k]);
        // cv::imshow(boost::filesystem::path(rgbFile[k]).stem().string(), cimg);
        // cv::imshow(boost::filesystem::path(depFile[k]).stem().string(), dimg);
    }
    cv::Mat rd2show;
    cv::hconcat(rgbdep[0], rgbdep[1], rd2show);
    cv::imshow("rd2show", rd2show);
    char c = (char)cv::waitKey(0);
    if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
    exit(-1);
}

void C2DSVDTransEstim::FramePair::Restore3d(const CamProj& camProj, const float& scale)
{
    szPoints = rgbPoints[0].size();
    for (int k = 0; k < 2; ++k)
    {
        centroid[k] = cv::Point3f(0, 0, 0);
        depPoints[k].resize(szPoints);
    }

    for (int k = 0; k < 2; ++k)
    {
        // cv::Mat clrimg;
        // cv::cvtColor(dep[k], clrimg, cv::COLOR_GRAY2BGR);
        bool badDepth = false;
        cv::Mat depsc;
        dep[k].convertTo(depsc, CV_32FC1);
        depsc *= scale;
        for (size_t ci = 0; ci < szPoints; ++ci)
        {
            const float y = rgbPoints[k][ci].x;
            const float x = rgbPoints[k][ci].y;
            const int x0 = std::floor(x+0.5), y0 = std::floor(y+0.5);
            if (0 > x0 || 0 > y0 || depsc.rows <= x0 || depsc.cols <= y0)
            {
                std::cout << "C2DSVDTransEstim::FramePair::Restore3d: (" << x0 << ", " << y0 << ") out of range (" << depsc.rows << ", " << depsc.cols << ")!\n";
                return;
            }
            cv::Point3f pos = camProj.Get3D(x0, y0);
            const float dd = depsc.at<float>(x0, y0);
            if (dd < 1) // this should not happen!
            {
                badDepth = true;
                std::cout << x0 << " " << y0 << ": ";
               for (int r = -2; r < 3; ++r)
               for (int c = -2; c < 3; ++c)
               {
                   const float v = depsc.at<float>(x0+r, y0+c);
                   // if (10000 < v || -10000 > v)
                   std::cout << v << " ";
               }
                std::cout << " -- " << dd << std::endl;
            }
            pos *= dd;
            depPoints[k][ci] = pos;
            centroid[k] += pos;
            // cv::circle(clrimg, cv::Point(x0, y0), 3, cv::Scalar(0, 0, 255), 1);
        }
        // imshow("cir", clrimg);
        if (badDepth)
        {
            // imshow("0", (dep[k] < 10));
            imshow("3", (depsc < 10));
            cv::waitKey(0);
        }
    }
    for (int k = 0; k < 2; ++k)
    {
        centroid[k] /= (float)szPoints;
    }
}

int C2DSVDTransEstim::BuildCheckerMap(void)
{
    if (image_list.size() % 2 != 0)
    {
        std::cout << "C2DSVDTransEstim::BuildCheckerMap: odd (non-even) number of image list\n";
        return -1;
    }
    if (image_list.size() != depth_list.size())
    {
        std::cout << "C2DSVDTransEstim::BuildCheckerMap: image/depth number not matching\n";
        return -1;
    }

    const int maxScale = 2;
    size_t nimages = (int)image_list.size()/2;
    size_t npairs = 0;
    cv::Size imgSize;
    for (size_t ii = 0; ii < nimages; ++ii)
    {
        FramePair framePair;
        int k;
        for (k = 0; k < 2; ++k)
        {
            const std::string& filename = image_list[ii*2+k];
            cv::Mat imgrd = cv::imread(filename, cv::IMREAD_COLOR);
            if (imgrd.empty())
            {
                std::cout << "C2DSVDTransEstim::BuildCheckerMap: image [" << filename << "] cannot be read!\n";
                break;
            }
            if (0 == k) imgSize = imgrd.size();
            else if (imgrd.size() != imgSize)
            {
                std::cout << "C2DSVDTransEstim::BuildCheckerMap: image [" << filename << "] has different size from the first\n";
                break;
            }
            framePair.rgbFile[k] = filename;
            framePair.rgb[k] = imgrd;
            bool found = false;
            std::vector<cv::Point2f>& corners = framePair.rgbPoints[k];
            for (int scale = 1; scale <= maxScale; ++scale)
            {
                cv::Mat timg;
                if( scale == 1 ) timg = imgrd;
                else cv::resize(imgrd, timg, cv::Size(), scale, scale);
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
                        cornersMat /= scale;
                    }
                    break;
                }
            }
            // if (!found) break;
            // this makes detected corners messy!
            // cv::cornerSubPix(
            //     imgrd, corners, cv::Size(11,11), cv::Size(-1,-1),
            //     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01)
            // );
        }
        if (2 > k) continue;
        for (k = 0; k < 2; ++k)
        {
            const std::string& filename = depth_list[ii*2+k];
            cv::Mat imgrd = cv::imread(filename, cv::IMREAD_ANYDEPTH);
            if (imgrd.empty())
            {
                std::cout << "C2DSVDTransEstim::BuildCheckerMap: image [" << filename << "] cannot be read!\n";
                break;
            }
            if (0 == k) imgSize = imgrd.size();
            else if (imgrd.size() != imgSize)
            {
                std::cout << "C2DSVDTransEstim::BuildCheckerMap: image [" << filename << "] has different size from the first!\n";
                break;
            }
            if (-1 == depth_bits)
            {
                depth_type = imgrd.depth();
                if (CV_8U == depth_type) depth_bits = 8;
                else if (CV_16U == depth_type) depth_bits = 16;
                else
                {
                    std::cout << "C2DSVDTransEstim::BuildCheckerMap: unsupported depth type!\n";
                    return -1;
                }
                std::cout << "depth images has [" << depth_bits << "] bits\n";
            } else if (imgrd.depth() != depth_type)
            {
                std::cout << "C2DSVDTransEstim::BuildCheckerMap: inconsistent depth bits [" << imgrd.depth() << "]!\n";
                return -1;
            }
            framePair.depFile[k] = filename;
            framePair.dep[k] = imgrd;
        }
        if (2 > k) continue;
        npairs++;
        framePair.szBoard = board_size;
        frames.push_back(framePair);
        if (display_corners)
        framePair.DrawCorners();
    }
    std::cout << "C2DSVDTransEstim::BuildCheckerMap: " << npairs << " pairs have been successfully detected.\n";

    const int maxDepth = (1<<depth_bits)-1;
    for (int k = 0; k < 2; ++k)
    {
        centroid[k] = cv::Point3f(0, 0, 0);
    }
    for (size_t nf = 0; nf < frames.size(); ++nf)
    {
        frames[nf].Restore3d(camera_matrix, scaleFactor/maxDepth);
        for (int k = 0; k < 2; ++k)
        {
            centroid[k] += frames[nf].centroid[k];
        }
    }
    for (int k = 0; k < 2; ++k)
    {
        centroid[k] /= (float)frames.size();
    }
    return 0;
}

int C2DSVDTransEstim::TestTrans(const std::string& outFile)
{
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    const FramePair& frame = frames[0];
    for (int k = 0; k < 2; ++k)
    {
        const cv::Mat& rgb = frame.rgb[k];
        cv::Mat dep = frame.dep[k].clone();
        const int maxDepth = (1<<depth_bits)-1;
        dep.convertTo(dep, CV_32FC1, scaleFactor/maxDepth);
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
                cv::Point3f pos = camProj.Get3D(ri, ci) * dij;
                if (0 == k)
                {
                    pos = rotate_l2r * pos + translate_l2r;
                }
                pos = pos / 1e3; // scale to meter
                pcl::PointXYZRGB point;
                point.x = pos.x;
                point.y = pos.y;
                point.z = pos.z;
                point.rgb = *reinterpret_cast<const float*>(&rgb.at<cv::Vec3b>(ri, ci));
                pointCloud.push_back(point);
            }
        }
    }
    pcl::io::savePLYFile(outFile, pointCloud);
}

int C2DSVDTransEstim::WriteTrans(const std::string& outFile)
{
    if (outFile.empty()) return -1;
    cv::FileStorage storage(outFile, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        std::cout << "C2DSVDTransEstim::WriteTrans: [" << outFile << "] not openned!";
        return -1;
    }
    storage << "R" << cv::Mat(rotate_l2r);
    storage << "T" << cv::Mat(translate_l2r);
    storage.release();
    return 0;
}
