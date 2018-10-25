
#include "PlatformDetector.h"
#include <sstream>

using namespace cv;
using namespace std;

PlatformDetector::PlatformDetector() {
    //mLogoGRVC = cv::Mat(50,60,CV_8UC3, logoGrvcData);
}

tracked_mark PlatformDetector::detect(Mat &_frame, int _minSize, int _maxSize, double _altitude){
    Mat frameOri = _frame.clone();

    // Check iamge
    if (_frame.empty())
        return tracked_mark();

    //crop search window if applicable
    Mat frameThresh(_frame.size(), CV_8UC1, Scalar(0));
    if (mMarkStatus == FOUND || mMarkStatus == TRACKED) {
        Mat croppedFrame;
        _frame(mSearchWindow).copyTo(croppedFrame);
        preproccessFrame(croppedFrame, mBestCandidate.mean, _altitude).copyTo(frameThresh(mSearchWindow));

    }else{
        frameThresh = preproccessFrame(_frame, mBestCandidate.mean,_altitude);
    }

    //search for candidates if mark is not found
    //evaluate them, and select the best one
    mBestCandidate = findMark(frameThresh, frameOri,  _minSize,  _maxSize);
    drawPoligon(mBestCandidate.contour, &frameOri, Scalar(0, 0, 255));

    //calculate window search (green) around the best candidate
    // in case there is no candidate, search again in the whole frame
    if (mMarkStatus == FOUND || mMarkStatus == TRACKED) {
        mSearchWindow = boundingRect(mBestCandidate.contour); // createSearchWindow(mBestCandidate.contour, frame_copy, mSearchWindow, &mark_shift);
        mSearchWindow.x -= mSearchWindow.width*0.5; mSearchWindow.y -= mSearchWindow.height*0.5;
        mSearchWindow.width *= 2; mSearchWindow.height *= 2;
        mSearchWindow &= cv::Rect(0,0,_frame.cols, _frame.rows);
        drawPoligon(mBestCandidate.contour, &frameOri, Scalar(0, 0, 255),Point(0,0));
        rectangle(frameOri, mSearchWindow, Scalar(0, 255, 0));
    }
    else {
        mSearchWindow = Rect(0, 0, frameOri.cols, frameOri.rows);
        rectangle(frameOri, mSearchWindow, Scalar(0, 255, 0));
    }

    //display results and count time
    switch (mMarkStatus) {
    case Status::FOUND:
        putText(frameOri, "FOUND", Point(10,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,255),3);
        break;
    case Status::TRACKED:
        putText(frameOri, "TRACKED", Point(10,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,255), 3);
        break;
    case Status::NOT_FOUND:
        putText(frameOri, "NOT_FOUND", Point(10,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,255), 3);
        break;
    }

    putText(frameOri, "Altitude: "+ std::to_string(_altitude), Point(10,80), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255), 2);
    
    //namedWindow("frame",CV_WINDOW_FREERATIO);
    //imshow("frame", frameOri);
    //waitKey(3);
    _frame = frameOri;
    return mBestCandidate;
}



//This function returns the 4 sided poligons found
// in the frame
vector<vector<Point> > PlatformDetector::findSquares(Mat &thresh, Mat &frameEdges, int _minSize, int _maxSize) {
    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(4, 4));
    //erode(thresh, thresh, element);

    //find contours on the frame
    Canny(thresh, frameEdges, 200, 1000);

    ///--->namedWindow("thresh",CV_WINDOW_FREERATIO);
    ///--->imshow("thresh", thresh);
    ///--->namedWindow("canny",CV_WINDOW_FREERATIO);
    ///--->imshow("canny", frameEdges);

    vector<vector<Point> > cnts;
    findContours(frameEdges, cnts, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS);
    //approximate contours to poligons
    vector<vector<Point> > candidates;
    Mat edges(frameEdges.size(), CV_8UC3, Scalar(0,0,0));
    std::stringstream ssCandidatesAreas;
    ssCandidatesAreas << "Found objects with following areas: ";
    for (int i = 0; i < cnts.size(); i++) {
        drawPoligon(cnts[i], &edges, Scalar(255,255,255),Point(0,0), false);

        std::vector<Point> hull;
        convexHull(cnts[i], hull);
        double peri = arcLength(hull, 1);
        vector<Point> poligon;
        approxPolyDP(hull, poligon, peri*0.1, 1);
        double area = contourArea(poligon);

        ssCandidatesAreas << area <<", ";

        if (area > _minSize*_minSize && area < _maxSize*_maxSize) {
            drawPoligon(cnts[i], &edges, Scalar(0,255,0),Point(0,0), false);
            drawPoligon(hull, &edges, Scalar(255,0,0),Point(0,0), false);
            drawPoligon(poligon, &edges, Scalar(0,0,255),Point(0,0), false);
            putText(edges, std::to_string(area), poligon[0], FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0));

            if (poligon.size() == 4) {
                candidates.push_back(poligon);
            }
        }
    }

    ///--->namedWindow("edges",CV_WINDOW_FREERATIO);
    ///--->imshow("edges", edges);

    //return the possible candidates
    if (candidates.empty()) {
        mMarkStatus = NOT_FOUND;
    }
    else
        return candidates;
}

//This function returns for each square a coefficent <=1 describing how parallel
//are the lines of the squares found
vector<float> PlatformDetector::extractSlopeCoefficent(vector<vector<Point> > candidates) {
    vector<float> slope_coefficents;
    for (int i = 0; i < candidates.size(); i++) {

        cv::Vec2f v1, v2, v3, v4;

        v1 = {(float) candidates[i][1].x - candidates[i][0].x, (float) candidates[i][1].y - candidates[i][0].y};
        v2 = {(float) candidates[i][2].x - candidates[i][1].x, (float) candidates[i][2].y - candidates[i][1].y};
        v3 = {(float) candidates[i][3].x - candidates[i][2].x, (float) candidates[i][3].y - candidates[i][2].y};
        v4 = {(float) candidates[i][0].x - candidates[i][3].x, (float) candidates[i][0].y - candidates[i][3].y};

        double ca1 = (v1[0]*v3[0] + v1[1]*v3[1])/(norm(v1)*norm(v3));
        double ca2 = (v2[0]*v4[0] + v2[1]*v4[1])/(norm(v2)*norm(v4));

        double midVal = (ca1+ca2)/2;
        if(midVal < 0) midVal *=-1;

        slope_coefficents.push_back(midVal);
    }
    return slope_coefficents;
}

//This function recieves an set of points and draws
//on the Mat "frame" the poligon they form
void PlatformDetector::drawPoligon(vector<Point> poligon, Mat *frame, Scalar color, Point offset, bool _ps) {
    for (int i = 0; i < poligon.size(); i++) {
        if(_ps)
            putText(*frame, "P", Point(poligon[i].x + offset.x, poligon[i].y + offset.y), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0));

        if (i<poligon.size() - 1)
            line(*frame, poligon[i] + offset, poligon[i + 1] + offset, color);
        else
            line(*frame, poligon[poligon.size() - 1] + offset, poligon[0] + offset, color);
    }

}

//This function draws the best candidate in red
// and the rest of them in blue
void PlatformDetector::drawCandidates(vector<vector<Point> > candidates, Mat * frame, int idx, Rect search_window) {
    if (!candidates.empty()) {
        Point offset = Point(search_window.x, search_window.y);

        for (int i = 0; i < candidates.size(); i++) {
            if (i != idx) drawPoligon(candidates[i], frame, Scalar(255, 0, 0), offset);
        }
        drawPoligon(candidates[idx], frame, Scalar(0, 0, 255), offset);
    }
}

//cross detection
//candidates can score up to 2 points if they have a line with negative slope and a line with positive
//slope close to the centroid
vector<int> PlatformDetector::detectCross(vector<struct candidate_region> regions, vector<vector<Point> > candidates, Mat * frame_edges) {
    vector<Vec2f> lines;
    Rect brect;
    vector<int> scores;
    //these flags indicate if a line with positive or negative slope
    //passes near the centroid of the region
    bool neg_slope;
    bool pos_slope;

    for (int i = 0; i < candidates.size(); i++) {
        //crop the fragment of the canny image bounded to the candidate
        brect = boundingRect(candidates[i]);
        Mat copy, crop;
        frame_edges->copyTo(copy);
        copy = copy(brect);
        copy.copyTo(crop, regions[i].mask);

        //calculate centroid of the region
        Moments M = moments(regions[i].mask);
        Point centroid(M.m10 / M.m00, M.m01 / M.m00);
        //cout << "DetectCross: Centroid: ( " << centroid.x << " , " << centroid.y << " )" << endl;

        //take the fraction on the center
        Rect zoom;
        zoom.x = centroid.x - 0.2 * crop.cols;
        zoom.y = centroid.y - 0.2 * crop.rows;
        zoom.width = 0.4*crop.cols;
        zoom.height = 0.4*crop.rows;
        Mat test = crop(zoom);
        crop = crop(zoom);
        Point gcenter(crop.cols/2,crop.rows/2);
        int zeros = countNonZero(crop);

        //find lines
        HoughLines(crop, lines, 1, (0.5*CV_PI / 180), 0.05*zeros);
        cvtColor(crop, crop, COLOR_GRAY2BGR);
        circle(crop, centroid, 5, Scalar(255, 0, 0), 2);
        circle(crop, gcenter, 5, Scalar(255, 0, 0), 2);
        //c---> cout << "DetectCross: " << lines.size() << "lines generated" << endl;

        //predict the slopes of the cross
        float slope, slope_neg = 0, slope_pos = 0;
        slope = (candidates[i][2].x - candidates[i][0].x) / (float)(candidates[i][2].y - candidates[i][0].y);
        if (slope < 0) {
            slope_neg = slope;
        }
        else {
            slope_pos = slope;
        }
        slope = (candidates[i][3].x - candidates[i][1].x) / (float)(candidates[i][3].y - candidates[i][1].y);
        if (slope < 0) {
            slope_neg = slope;
        }
        else {
            slope_pos = slope;
        }

        //these two flags indicate if the line detected is close to the centroid AND
        neg_slope = false;
        pos_slope = false;

        for (size_t u = 0; u < lines.size(); u++) {
            float rho = lines[u][0], theta = lines[u][1];
            //cout << "Line " << u << " : theta = " << theta << ", rho = " << rho << endl;
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);

            if ((!pos_slope && a > 0) || (!neg_slope && a < 0)) {
                double x0 = a*rho, y0 = b*rho;
                pt1.x = cvRound(x0 + 1000 * (-b)); pt1.y = cvRound(y0 + 1000 * (a));
                pt2.x = cvRound(x0 - 1000 * (-b)); pt2.y = cvRound(y0 - 1000 * (a));

                if (a < 0) line(crop, pt1, pt2, Scalar(0, 0, 255), 1, CV_AA);
                else line(crop, pt1, pt2, Scalar(0, 255, 0), 1, CV_AA);

                //Check if the line is close to the centroid
                double dist = distanceL2P(pt1, pt2, gcenter);
                //cout << "Distance: " << dist << endl;
                if (dist < 0.05*crop.cols) {
                    if (a > 0) {
                        pos_slope = true;
                    }
                    if (a < 0) {
                        neg_slope = true;
                    }
                    line(crop, pt1, pt2, Scalar(0, 255, 255), 4, CV_AA);
                }
                line(*frame_edges, candidates[i][1], candidates[i][3], Scalar(200, 200, 200), 4, CV_AA);
                line(*frame_edges, candidates[i][0], candidates[i][2], Scalar(200, 200, 200), 4, CV_AA);
            }
        }
        //--> imshow("cross detection lines", crop);
        //imshow("cross localizations", *frame_edges);

        //score +1 point for each line close to the centroid detected
        int temp_score = 0;
        if (pos_slope) temp_score++;
        if (neg_slope) temp_score++;
        scores.push_back(temp_score);
    }
    return scores;
}

//This function calculates the distance between a line (given 2 points) and an external point
double PlatformDetector::distanceL2P(Point line_begin, Point line_end, Point point) {
    vector<Point> triangle;
    triangle.push_back(line_begin);
    triangle.push_back(line_end);
    triangle.push_back(point);
    triangle.push_back(line_begin);

    float area = contourArea(triangle);
    double base = norm(line_end - line_begin);
    double distance = 2 * area / base; /*From the triangle area formula*/

    triangle.clear();
    return distance;
}

//Preproccessing function to make contours clear
Mat PlatformDetector::preproccessFrame(Mat _frame, double _candidateMean, double _altitude) {
    Mat frame_gray, frame_thresh;

    if(_altitude < 5){
        GaussianBlur(_frame, _frame, Size(11, 11), 4);
    }else if(_altitude < 14){
        GaussianBlur(_frame, _frame, Size(5, 5), 4);
    }else if(_altitude >= 14){
        GaussianBlur(_frame, _frame, Size(3, 3), 4);
    }
    cvtColor(_frame, frame_gray, COLOR_BGR2GRAY);
    threshold(frame_gray, frame_thresh,200,255 , THRESH_BINARY);

    return frame_thresh;
}

tracked_mark PlatformDetector::findMark(Mat frame_thresh, Mat frame, int _minSize, int _maxSize) {
    struct tracked_mark best_candidate;

    //Search for 4-sided poligons (candidates)
    vector<vector<Point> > candidates;
    Mat frame_edges;
    candidates = findSquares(frame_thresh, frame_edges,_minSize,  _maxSize);
    for (int i = 0; i < candidates.size(); i++) {
        drawPoligon(candidates[i], &frame, Scalar(255, 0, 0));
    }
    //Extract slope coefficents
    vector<float> slope_coefficents;
    slope_coefficents = extractSlopeCoefficent(candidates);
    //eliminate candidates with low slope coefficents
    for (int i = 0; i < slope_coefficents.size(); i++) {
        //std::cout << slope_coefficents[i]<< std::endl;
        if (slope_coefficents[i] < 0.3) {
            slope_coefficents.erase(slope_coefficents.begin()+i);
            candidates.erase(candidates.begin() + i);
            i--;
        }
    }

    //Extract regions and masks
    vector<struct candidate_region> regions;
    regions = extractRegions(candidates, &frame_edges, &frame);

    //Score means
    vector<double> region_means;
    region_means = extractRegionMean(regions);
    //eliminate candidates with low slope coefficents
    for (int i = 0; i < region_means.size(); i++) {
        if (region_means[i] < 0.5) {
            region_means.erase(region_means.begin() + i);
            regions.erase(regions.begin() + i);
            candidates.erase(candidates.begin() + i);
            i--;
        }
    }
    //Find crosses
    vector<int> crosses;
    crosses = detectCross(regions, candidates, &frame_edges);

    //Evaluate each candidate
    //take the candidate with the highest score and check if it reaches a minimum score
    float highest_score = 0, temp_score;
    int idx = -1;
    for (int i = 0; i < candidates.size(); i++) {
        //draw on frame individual scores
        std::stringstream ss;
        ss << "Slope: " << slope_coefficents[i];
        putText(frame, ss.str(), candidates[i][1], CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
        ss << "Mean: " << region_means[i];
        putText(frame, ss.str(), Point(candidates[i][1].x, candidates[i][1].y-20), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));

        temp_score = slope_coefficents[i] + region_means[i] + crosses[i];
        if (temp_score > highest_score) {
            highest_score = temp_score;
            idx = i;
        }
    }

    //if none of the candidates passes the minimun score
    //then we suppose the mark is not found
    if (highest_score < 1.5) {
        best_candidate.isvalid = false;
        mMarkStatus = NOT_FOUND;
    }
    else {
        //If a good candidate is found, return its data
        best_candidate.contour = candidates[idx];
        best_candidate.mean  = region_means[idx];
        best_candidate.area  = contourArea(candidates[idx]);
        best_candidate.isvalid = true;
        mMarkStatus = FOUND;
    }

    return best_candidate;
}

//This function returns a set of images with the isolated region enclosed by each of the candidates
//and their respective masks
vector<struct candidate_region> PlatformDetector::extractRegions(vector<vector<Point> > candidates, Mat * canny, Mat * original_frame) {
    vector<struct candidate_region> regions;
    struct candidate_region temp_region;

    for (int i = 0; i < candidates.size(); i++) {
        vector < vector<Point> > candidate_v;
        candidate_v.push_back(candidates[i]);

        //Create and store mask to isolate the candidate
        Rect roi = boundingRect(candidates[i]);
        Mat mask = Mat::zeros(canny->size(), CV_8UC1);
        drawContours(mask, candidate_v, -1, Scalar(255), CV_FILLED);

        //Crop the isolated region
        Mat contour_region;
        original_frame->copyTo(contour_region,mask);
        contour_region = contour_region(roi);

        //Add it to the vector of regions
        temp_region.region = contour_region;
        temp_region.mask   = mask(roi);
        regions.push_back(temp_region);

        candidate_v.clear();
    }
    return regions;
}

//This function takes the mean value of each of the squares found.
//A high mean scores more points than a low mean.
vector<double> PlatformDetector::extractRegionMean(vector<struct candidate_region> candidate_regions) {
    vector<double> region_means;
    for (int i = 0; i < candidate_regions.size(); i++) {
        //Store the normalized value of mean in the output vector
        Scalar temp_value = mean(candidate_regions[i].region, candidate_regions[i].mask);
        region_means.push_back(temp_value[2] / 255);
    }
    return region_means;
}
