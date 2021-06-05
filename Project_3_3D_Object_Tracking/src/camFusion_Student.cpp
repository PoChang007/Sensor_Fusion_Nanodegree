
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::resize(topviewImg, topviewImg, cv::Size(960, 900));
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    vector<cv::KeyPoint> qualifyKeypoints;
    vector<cv::DMatch> qualifyMatches;
    vector<double> featureDistance;
    vector<int> matchCandidateIndex;
    vector<double> matchDistanceFromMean;
    double distanceSum = 0.0;

    for (auto it_match = kptMatches.begin(); it_match != kptMatches.end(); ++it_match)
    {
        auto currKeyPointPos = kptsCurr[it_match->trainIdx].pt;
        auto prevKeyPointPos = kptsPrev[it_match->queryIdx].pt;
        if (boundingBox.roi.contains(currKeyPointPos) && boundingBox.roi.contains(prevKeyPointPos))
        {
            auto matchIndex = std::distance(kptMatches.begin(), it_match);
            auto distance = (double)it_match->distance;
            featureDistance.push_back(distance);
            matchCandidateIndex.push_back(matchIndex);
            distanceSum += distance;
        }
    }

    double distanceMean = distanceSum / (double)matchCandidateIndex.size();

    double squareDistanceSumCurr = 0.0;
    for (int i = 0; i < featureDistance.size(); ++i)
    {
        auto distance = fabs(featureDistance[i] - distanceMean) * fabs(featureDistance[i] - distanceMean);  
        squareDistanceSumCurr += distance;
        matchDistanceFromMean.push_back(sqrt(distance));
    }

    auto sdMatchDistance = sqrt(squareDistanceSumCurr / (double)featureDistance.size());

    for (int i = 0; i < matchCandidateIndex.size(); ++i)
    {
        if (kptMatches[matchCandidateIndex[i]].distance < distanceMean)
        {
            qualifyMatches.push_back(kptMatches[matchCandidateIndex[i]]);
            qualifyKeypoints.push_back(kptsCurr[kptMatches[matchCandidateIndex[i]].trainIdx]);
        }
    }

    boundingBox.keypoints = qualifyKeypoints;
    boundingBox.kptMatches = qualifyMatches;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { 
        // outer keypoint loop
        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { 
            // inner keypoint loop
            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { 
                // avoid division by zero
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double dT = 1 / frameRate;
    std::sort(distRatios.begin(), distRatios.end());
    if (distRatios.size() % 2 == 1)
    {
        double medianDistRatio = distRatios[distRatios.size()/2 + 1];
        TTC = -dT / (1 - medianDistRatio);
    }
    else
    {
        double medianDistRatio = (distRatios[distRatios.size()/2] + distRatios[distRatios.size()/2 + 1]) / 2.0f;
        TTC = -dT / (1 - medianDistRatio);
    }
    std::cout << "Camera TTC is " << TTC << "\n";
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1/frameRate; // time between two measurements in seconds
    double laneWidth = 4.0;  // assumed width of the ego lane
    std::vector<LidarPoint> lidarPointCandidatePrev;
    std::vector<LidarPoint> lidarPointCandidateCurr;
    std::vector<double> prevX;
    std::vector<double> currX;

    // filter out possible outliers
    // calculate mean and standard deviation of detected lidar points
    vector<double> lidarPointPrevMean = {0.0, 0.0, 0.0, 0.0};
    vector<double> lidarPointCurrMean = {0.0 ,0.0, 0.0, 0.0};
    vector<double> lidarPointPrevDistanceFromMean;
    vector<double> lidarPointCurrDistanceFromMean;
    double sdScale = 1.0;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (it->y < (laneWidth/2.0) && it->y > -(laneWidth/2.0))
        {
            lidarPointCandidatePrev.push_back((*it));
            lidarPointPrevMean[0] += it->x;
            lidarPointPrevMean[1] += it->y;
            lidarPointPrevMean[2] += it->z;
            lidarPointPrevMean[3] += 1;
        }
    }

    lidarPointPrevMean[0] /= lidarPointPrevMean[3];
    lidarPointPrevMean[1] /= lidarPointPrevMean[3];
    lidarPointPrevMean[2] /= lidarPointPrevMean[3];

    double squareDistanceSumPrev = 0.0;
    for (auto it_prev = lidarPointCandidatePrev.begin(); it_prev != lidarPointCandidatePrev.end(); ++it_prev)
    {
        auto deltaX = fabs(it_prev->x - lidarPointPrevMean[0]);
        auto deltaY = fabs(it_prev->y - lidarPointPrevMean[1]);
        auto deltaZ = fabs(it_prev->z - lidarPointPrevMean[2]);
        auto distance = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
        squareDistanceSumPrev += distance;
        lidarPointPrevDistanceFromMean.push_back(sqrt(distance));
    }
    auto sdPrev = sqrt(squareDistanceSumPrev / (double)lidarPointCandidatePrev.size());

    for (auto it_prev = lidarPointCandidatePrev.begin(); it_prev != lidarPointCandidatePrev.end(); ++it_prev)
    {
        auto index = std::distance(lidarPointCandidatePrev.begin(), it_prev);
        if (lidarPointPrevDistanceFromMean[index] <= sdScale * sdPrev)
        {
            prevX.push_back(it_prev->x);
        }
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (it->y < (laneWidth/2.0) && it->y > -(laneWidth/2.0))
        {
            lidarPointCandidateCurr.push_back((*it));
            lidarPointCurrMean[0] += it->x;
            lidarPointCurrMean[1] += it->y;
            lidarPointCurrMean[2] += it->z;
            lidarPointCurrMean[3] += 1;
        }
    }

    lidarPointCurrMean[0] /= lidarPointCurrMean[3];
    lidarPointCurrMean[1] /= lidarPointCurrMean[3];
    lidarPointCurrMean[2] /= lidarPointCurrMean[3];

    double squareDistanceSumCurr = 0.0;
    for (auto it_curr = lidarPointCandidateCurr.begin(); it_curr != lidarPointCandidateCurr.end(); ++it_curr)
    {
        auto deltaX = fabs(it_curr->x - lidarPointCurrMean[0]);
        auto deltaY = fabs(it_curr->y - lidarPointCurrMean[1]);
        auto deltaZ = fabs(it_curr->z - lidarPointCurrMean[2]);
        auto distance = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
        squareDistanceSumCurr += distance;
        lidarPointCurrDistanceFromMean.push_back(sqrt(distance));
    }
    auto sdCurr = sqrt(squareDistanceSumCurr / (double)lidarPointCandidateCurr.size());

    for (auto it_curr = lidarPointCandidateCurr.begin(); it_curr != lidarPointCandidateCurr.end(); ++it_curr)
    {
        auto index = std::distance(lidarPointCandidateCurr.begin(), it_curr);
        if (lidarPointCurrDistanceFromMean[index] <= sdScale * sdCurr)
        {
            currX.push_back(it_curr->x);
        }
    }

    // find the average distance of the qualified Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;

    minXPrev = std::accumulate(prevX.begin(), prevX.end(), 0.0f) / (double)prevX.size();
    minXCurr = std::accumulate(currX.begin(), currX.end(), 0.0f) / (double)currX.size();

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
    std::cout << "Lidar TTC is " << TTC << "\n";
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    auto prevKeyPoints = prevFrame.keypoints;
    auto currKeyPoints = currFrame.keypoints;
    vector<int> prevBoundIndex;
    vector<int> currBoundIndex;
    vector<int> prevBoxCandidate;
    vector<int> currBoxCandidate;
    vector<int> matchSizeInAssociatedBoxes;

    // count the number of matching keypoints for each pair of bounding boxes
    for (auto it_x = prevFrame.boundingBoxes.begin(); it_x != prevFrame.boundingBoxes.end(); ++it_x)
    {
        auto prevBoundingIndex = std::distance(prevFrame.boundingBoxes.begin(), it_x);

        for (auto it_y = currFrame.boundingBoxes.begin(); it_y != currFrame.boundingBoxes.end(); ++it_y)
        {
            auto currBoundingIndex = std::distance(currFrame.boundingBoxes.begin(), it_y);
            prevBoundIndex.push_back(prevBoundingIndex);
            currBoundIndex.push_back(currBoundingIndex);

            auto match_size = 0;
            for (auto it_match = matches.begin(); it_match != matches.end(); ++it_match)
            {
                auto matchIndex = std::distance(matches.begin(), it_match);
                auto prevKeyPointPos = prevKeyPoints[it_match->queryIdx].pt;
                auto currKeyPointPos = currKeyPoints[it_match->trainIdx].pt;

                if (it_x->roi.contains(prevKeyPointPos) && it_y->roi.contains(currKeyPointPos))
                {
                    match_size += 1;
                }
            }
            matchSizeInAssociatedBoxes.push_back(match_size);
        }
    }

    // pick up associated bounding boxes between the current frame and the previous frame
    auto prevBoundingSize = prevFrame.boundingBoxes.size();
    auto currBoundingSize = currFrame.boundingBoxes.size();

    while (prevBoundingSize != 0 && currBoundingSize != 0)
    {
        // choose the pair with the highest number of keypoints matches
        auto it_max_keypoints = std::max_element(matchSizeInAssociatedBoxes.begin(), matchSizeInAssociatedBoxes.end());
        if ((*it_max_keypoints) < 3) break;
        auto max_keypoints_index = std::distance(matchSizeInAssociatedBoxes.begin(), it_max_keypoints);

        auto it_search_prev_box = std::find(prevBoxCandidate.begin(), prevBoxCandidate.end(), prevBoundIndex[max_keypoints_index]);
        auto it_search_curr_box = std::find(currBoxCandidate.begin(), currBoxCandidate.end(), currBoundIndex[max_keypoints_index]);
        // auto it_search_prev_box = std::find_if(boxCandidate.begin(), boxCandidate.end(), [&test](std::pair<int, int> const& elem) { return elem.first == test;});       
        auto isSameClass = prevFrame.boundingBoxes[prevBoundIndex[max_keypoints_index]].classID == currFrame.boundingBoxes[currBoundIndex[max_keypoints_index]].classID;
        if (it_search_prev_box == prevBoxCandidate.end() && it_search_curr_box == currBoxCandidate.end() && isSameClass)
        {
            prevBoxCandidate.push_back(prevBoundIndex[max_keypoints_index]);
            currBoxCandidate.push_back(currBoundIndex[max_keypoints_index]);
            bbBestMatches.insert(std::make_pair(prevBoundIndex[max_keypoints_index], currBoundIndex[max_keypoints_index]));
            // std::cout << "combination: " << "\n";
            // std::cout << "prev " << prevBoundIndex[max_keypoints_index] << "\n";
            // std::cout << "curr " << currBoundIndex[max_keypoints_index] << "\n";

            matchSizeInAssociatedBoxes[max_keypoints_index] = 0;
            prevBoundingSize--;
            currBoundingSize--;
        }
        else
        {
            matchSizeInAssociatedBoxes[max_keypoints_index] = 0;
        }
    }
}
