#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
namespace zjrobot{
class RealsenseDriver{
public:
    RealsenseDriver(){
        m_realsense_cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        m_realsense_cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        m_realsense_pipe_.start(m_realsense_cfg_);
    }
    ~RealsenseDriver(){
        m_realsense_pipe_.stop();
    }
    bool WaitImages(cv::Mat& image, cv::Mat& depth){
        m_realsense_frames_ = m_realsense_pipe_.wait_for_frames();
        image.data = (uchar *)m_realsense_frames_.get_color_frame().get_data();
        depth.data = (uchar *)m_realsense_frames_.get_depth_frame().get_data();
        return true;
    }
private:
    rs2::config m_realsense_cfg_;
    rs2::pipeline m_realsense_pipe_;
    rs2::frameset m_realsense_frames_;
};

}
int main ( int argc, char** argv )
{
    zjrobot::RealsenseDriver camera;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descripts;
    cv::Ptr<cv::FeatureDetector> dector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //-- 读取图像
    while(1)
    {
        cv::Mat image(cv::Size(640, 480), CV_8UC3, cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(640, 480), CV_16U, cv::Mat::AUTO_STEP);
        camera.WaitImages(image, depth);
        dector->detect(image,keypoints);
        descriptor->compute(image, keypoints, descripts);
        cv::Mat out_img;
        cv::drawKeypoints(image, keypoints, out_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("keypoints", out_img);
        //cv::imshow("realsense depth", depth);
        int key = cv::waitKey(1);
        if(key == 'q')
            break;
    }
    cv::destroyAllWindows();

    return 0;
}
