#ifndef C2D_SVD_TRANS_ESTIM
#define C2D_SVD_TRANS_ESTIM

#include <opencv2/core.hpp>
#include <deque>

class C2DSVDTransEstim
{
public:
    struct CamProj
    {
        CamProj(void);
        CamProj(const cv::Mat& cammat);
        cv::Point3f Get3D(const int& r, const int& c) const;
    private:
        float fx, fy, cx, cy;
    };
    struct FramePair
    {
        std::string rgbFile[2], depFile[2];
        cv::Mat rgb[2], dep[2];
        cv::Size szBoard;
        size_t szPoints;
        std::vector< cv::Point2f > rgbPoints[2];
        std::vector< cv::Point3f > depPoints[2];
        cv::Point3f centroid[2];
        void DrawCorners(void);
        void Restore3d(const CamProj& camProj, const float& scale);
    };

public:
    C2DSVDTransEstim(void);
    int Calibrate(const std::string& configFile, const std::string& outFile = "");

public:
    int ReadConfig(const std::string& configFile);
    int BuildCheckerMap(void);
    int ComputeTransSVD(void);
    int TestTrans(const std::string& outFile);
    int WriteTrans(const std::string& outFile);

private:
    cv::Size board_size;
    float scaleFactor;
    int depth_bits;
    int depth_type;
    std::deque< FramePair > frames;
    std::deque< std::string > image_list;
    std::deque< std::string > depth_list;
    cv::Point3f centroid[2];
    cv::Mat camera_matrix;
    CamProj camProj;
    cv::Matx33f rotate_l2r;
    cv::Point3f translate_l2r;
    bool display_corners;
};

#endif
