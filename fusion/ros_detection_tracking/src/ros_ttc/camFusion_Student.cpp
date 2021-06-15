
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <tuple>
#include "../../include/ros_detection_tracking/camFusion.hpp"
#include "../../include/ros_detection_tracking/dataStructures.h"
//#include "utils.cpp"

using namespace std;
int findMedianIndex(int start_idx, int end_idx) 
{ 
    int range = (end_idx - start_idx)+1;
    // cout << "range = " << range << endl;
    
    if(range%2 != 0){
        return start_idx + range/2; 
    }
    else{
        return (start_idx + range/2)-1;
    }
}

std::tuple<double, double> IQRcam(std::vector<std::pair<int, double>> &inVec) 
{   
    double Q1, Q3;
    
    if(inVec.size()%2 != 0)
    {
        // Index of findMedianIndex of entire data 
        int mid_index = findMedianIndex(0, inVec.size()-1); 
        // cout << "Mid-Index: " << mid_index << endl;
        
        if(mid_index%2 != 0)
        {
            // Median of first half 
            Q1 = inVec[findMedianIndex(0, mid_index-1)].second; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            Q3 = inVec[findMedianIndex(mid_index + 1, inVec.size()-1)].second;
            // cout << "Q3: " << Q3 << endl;
        }
        else
        {
            // Median of first half 
            int q1_prev = findMedianIndex(0, mid_index - 1);
            Q1 = (double)(inVec[q1_prev].second + inVec[q1_prev+1].second)/2; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            int q3_prev = findMedianIndex(mid_index + 1, inVec.size()-1); 
            Q3 = (double)(inVec[q3_prev].second + inVec[q3_prev+1].second)/2;
            // cout << "Q3: " << Q3 << endl;
        }
    }
    else
    {
        int mid_index = findMedianIndex(0, inVec.size()-1);
        // cout << "Mid-Index: " << mid_index << endl;
        
        if((mid_index+1)%2 != 0)
        {
            // Median of first half 
            Q1 = inVec[findMedianIndex(0, mid_index)].second; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            Q3 = inVec[findMedianIndex(mid_index + 1, inVec.size()-1)].second;
            // cout << "Q3: " << Q3 << endl;
        }
        else
        {
            // Median of first half 
            int q1_prev = findMedianIndex(0, mid_index);
            Q1 = (double)(inVec[q1_prev].second + inVec[q1_prev+1].second)/2; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            int q3_prev = findMedianIndex(mid_index + 1, inVec.size()-1); 
            Q3 = (double)(inVec[q3_prev].second + inVec[q3_prev+1].second)/2;
            // cout << "Q3: " << Q3 << endl;
        }
    }
    
    double iqr = Q3 - Q1;
    
    double low_out = Q1 - 1.2*iqr;
    double high_out = Q3  + 1.2*iqr;
    
    // std::cout << "High : " << high_out << std::endl << "Low : " << low_out << std::endl << "IQR : " << iqr << std::endl;
    // IQR calculation 
    return std::make_tuple(low_out, high_out); 
}

void removeOutliersCam(std::vector<std::pair<int, double>> &distMap)
{
    // std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint p1, LidarPoint p2) -> bool{ return p1.x < p2.x; });
    std::sort(distMap.begin(), distMap.end(), [] (const std::pair<int,double> &a, const std::pair<int,double> &b) -> bool { return a.second < b.second; });
    
    auto minMaxBound = IQRcam(distMap);

    double lower_bound, upper_bound;
    std::tie(lower_bound, upper_bound) = minMaxBound;

    distMap.erase(std::remove_if(distMap.begin(), distMap.end(), [lower_bound, upper_bound](const std::pair<int, double> &ele) { return (ele.second < lower_bound || ele.second > upper_bound); }), distMap.end());
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, pcl::PointCloud<pcl::PointXYZ>::Ptr lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it1 = lidarPoints->points.begin(); it1 != lidarPoints->points.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

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
      std::vector<std::pair<int, double>> distMap;

    for(auto& match: kptMatches)
    {
        cv::KeyPoint prevKpt = kptsPrev[match.queryIdx];
    	cv::KeyPoint currKpt = kptsCurr[match.trainIdx];

        if(boundingBox.roi.contains(currKpt.pt))
        {
            double euDist = sqrt(pow(prevKpt.pt.x - currKpt.pt.x, 2) + pow(prevKpt.pt.y - currKpt.pt.y, 2));
            distMap.push_back(std::make_pair(match.trainIdx, euDist));
            // distances.push_back(euDist);
        }
    }

    // std::cout << "distMap size before outlier removal:" << distMap.size() << std::endl;

    removeOutliersCam(distMap);

    // std::cout << "distMap size after outlier removal: "<< distMap.size() << std::endl; 
    
    // int uni_count = std::distance(distMap.begin(), std::unique(distMap.begin(), distMap.end()));
    // int uni_count = std::distance(kptMatches.begin(), std::unique(kptMatches.begin(), kptMatches.end(), [](const cv::DMatch &a, const cv::DMatch &b) { return (a.trainIdx == b.trainIdx); } ));
    // std::cout << "Unique matches: " << uni_count << std::endl;

    for(auto ele : distMap)
    {
        // std::cout << ele.first << " " << ele.second << std::endl;
        for(auto match : kptMatches)
        {
            if(ele.first == match.trainIdx)
            {
                // std::cout << ele.first << " " << match.trainIdx << std::endl; 
                boundingBox.kptMatches.push_back(match);
            }
        }
    }
}


// // Compute time-to-collision (TTC) based on keypoint correspondences in successive images
// void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
//                       std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
// {
//    // compute distance ratios between all matched keypoints
//     vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
//     for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
//     { // outer kpt. loop

//         // get current keypoint and its matched partner in the prev. frame
//         cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
//         cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

//         for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
//         { // inner kpt.-loop

//             double minDist = 100.0; // min. required distance

//             // get next keypoint and its matched partner in the prev. frame
//             cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
//             cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

//             // compute distances and distance ratios
//             double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
//             double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

//             if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
//             { // avoid division by zero

//                 double distRatio = distCurr / distPrev;
//                 distRatios.push_back(distRatio);
//             }
//         } // eof inner loop over all matched kpts
//     }     // eof outer loop over all matched kpts

//     // only continue if list of distance ratios is not empty
//     if (distRatios.size() == 0)
//     {
//         TTC = NAN;
//         return;
//     }


//     // STUDENT TASK (replacement for meanDistRatio)
//     std::sort(distRatios.begin(), distRatios.end());
//     long medIndex = floor(distRatios.size() / 2.0);
//     double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

//     double dT = 1 / frameRate;
//     TTC = -dT / (1 - medDistRatio);
//     // EOF STUDENT TASK
// }


void computeTTCLidar(std::vector<pcl::PointXYZ> &lidarPointsPrev,
                     std::vector<pcl::PointXYZ> &lidarPointsCurr, double frameRate, double &TTC, bool simple)
{
   double dT = 1.0/frameRate;
    double meanXPrev, meanXCurr;
     if(simple)
    {
        ////简单方法计算
        // auxiliary variables
        double dT = 0.1; // time between two measurements in seconds

        // find closest distance to Lidar points 
        double minXPrev = 1e9, minXCurr = 1e9;
        for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
            minXPrev = minXPrev>it->x ? it->x : minXPrev;
        }

        for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
            minXCurr = minXCurr>it->x ? it->x : minXCurr;
        }

        // compute TTC from both measurements
        TTC = minXCurr * dT / (minXPrev-minXCurr);
    }
    // else 
    // {
    //         // std::cout << "lidarPointsPrev size = " << lidarPointsPrev.size() << endl;
    //     removeOutliersLidar(lidarPointsPrev);
    //     // std::cout << "lidarPointsPrev size after outlier removal = " << lidarPointsPrev.size() << endl;
        
    //     // std::cout << "lidarPointsCurr size = " << lidarPointsCurr.size() << endl;
    //     removeOutliersLidar(lidarPointsCurr);
    //     // std::cout << "lidarPointsCurr size after outlier removal = " << lidarPointsCurr.size() << endl;

    //     double prevTotal = 0, currTotal = 0;

    //     prevTotal = std::accumulate(lidarPointsPrev.begin(), lidarPointsPrev.end(), 0.0, [](double sum, const LidarPoint &pt){ return (sum + pt.x); } );
    //     meanXPrev = prevTotal / lidarPointsPrev.size();

    //     // std::cout << "meanXPrev: " << meanXPrev << std::endl;

    //     currTotal = std::accumulate(lidarPointsCurr.begin(), lidarPointsCurr.end(), 0.0, [](double sum, const LidarPoint &pt){ return (sum + pt.x); } );    
    //     meanXCurr = currTotal / lidarPointsCurr.size();

    //     // compute TTC from both measurements
    //     TTC = (meanXCurr * dT) / (meanXPrev - meanXCurr);
    // }

    
   
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
     const int size_p = prevFrame.boundingBoxes.size(); //检测框的数量
    const int size_c = currFrame.boundingBoxes.size();
    //int count[size_p][size_c] = {};//initialize a null matrix with all values "0" of bounding boxes size prev x curr
    cv::Mat count = cv::Mat::zeros(size_p, size_c, CV_32S);//通道默认为1
    for (auto matchpair : matches)  //每次取出一组对应特征点对
    {
        //take one matched keypoint at a time find the corresponsing point in current and prev frame
        //once done check to which bounding box in prev and curr frame the point belong too
        //once found store the value and increment the count
        //cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);
        cv::KeyPoint prevkp1 = prevFrame.keypoints.at(matchpair.queryIdx);//queryIdx代表的是前一帧特征点的索引
        auto prevkp = prevkp1.pt;//previous frame take keypint            //特征点坐标

        cv::KeyPoint currkp1 = currFrame.keypoints.at(matchpair.trainIdx);//trainIdx代表的是后一帧特征点的索引
        auto currkp = currkp1.pt;//current frame take keypint            ///特征点坐标

        for (size_t prevbb = 0; prevbb < size_p; prevbb++)//loop through all the prev frame bb
        {
            if (prevFrame.boundingBoxes[prevbb].roi.contains(prevkp))//check if the "previous frame take keypint" belongs to this box
            {//if it does
                for (size_t currbb = 0; currbb < size_c; currbb++)//loop thrpugh all the curr frame bb
                {
                    if (currFrame.boundingBoxes[currbb].roi.contains(currkp))//check if the "current frame take keypint" belongs to this box
                    {//if it does
                       
                        count.at<int>(prevbb, currbb) = count.at<int>(prevbb, currbb) + 1; 

                        //count.at<int>(prevFrame.boundingBoxes[prevbb].boxID, currFrame.boundingBoxes[currbb].boxID)  += 1;
                    }
                }
            }
        }
    }
    //for each prev bb find and compare the max count of corresponding curr bb.
    //the curr bb with max no. of matches (max count) is the bbestmatch

        for (size_t i = 0; i < size_p; i++)//loop through prev bounding box
        {
            int id = -1;//initialize id as the matrix starts from 0 x 0 we do not want to take 0 as the initializing value
            int maxvalue = 0;//initialize max value
            for (size_t j = 0; j < size_c; j++)//loop through all curr bounding boxes to see which prev + curr bb pair has maximum count
            {
                if (count.at<int>(i,j) > maxvalue)
                {
                    maxvalue = count.at<int>(i,j);//input value for comparison
                    id = j;//id
                }

            }
            bbBestMatches[i] = id;//once found for 1 prev bounding box; input the matched pair in bbBestMatches
        }
}
