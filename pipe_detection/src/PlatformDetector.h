
#ifndef PLATFORMTRACKER_PLATFORMDETECTOR_H_
#define PLATFORMTRACKER_PLATFORMDETECTOR_H_

#include <opencv2/opencv.hpp>

struct candidate_region {
    cv::Mat region;
    cv::Mat mask;
};

struct tracked_mark {
    std::vector<cv::Point> contour;
    int area;
    double mean;
    bool isvalid = false;
};

class PlatformDetector{
public:
    PlatformDetector();
    tracked_mark detect(cv::Mat &_frame, int _minSize, int _maxSize, double _altitude);

private:
    enum Status { FOUND, NOT_FOUND, TRACKED };
    Status mMarkStatus = NOT_FOUND;

    const double MIN_SCORE = 1.5;
    cv::Rect mSearchWindow;
    struct tracked_mark mBestCandidate;

private:
    cv::Mat preproccessFrame(cv::Mat _frame, double _candidateMean, double _altitude);
    //Additional functions
    std::vector<std::vector<cv::Point> >   findSquares(cv::Mat &thresh, cv::Mat &frameEdges, int _minSize, int _maxSize);
    struct tracked_mark     findMark(cv::Mat frame_thresh, cv::Mat frame, int _minSize, int _maxSize);
    std::vector<struct candidate_region> extractRegions(std::vector<std::vector<cv::Point> > candidates, cv::Mat * canny, cv::Mat * original_frame);
    std::vector<double>     extractRegionMean(std::vector<struct candidate_region> candidate_regions);
    std::vector<float>      extractSlopeCoefficent(std::vector<std::vector<cv::Point> > candidates);
    std::vector<int>        detectCross(std::vector<struct candidate_region> regions, std::vector<std::vector<cv::Point> > candidates, cv::Mat * frame_edges);
    void                    drawPoligon(std::vector<cv::Point> poligon, cv::Mat *frame, cv::Scalar color, cv::Point offset = cv::Point(0,0), bool _ps = true);
    void                    drawCandidates(std::vector<std::vector<cv::Point> > candidates, cv::Mat * frame, int idx, cv::Rect mSearchWindow = cv::Rect());
    double                  distanceL2P(cv::Point line_begin, cv::Point line_end, cv::Point point);
};

#endif