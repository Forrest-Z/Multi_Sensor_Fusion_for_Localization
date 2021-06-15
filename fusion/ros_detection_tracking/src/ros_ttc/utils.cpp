#include <vector>
#include <algorithm>
#include <tuple>

#include "../../include/ros_detection_tracking/dataStructures.h"

// Function to give index of the median 
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

std::tuple<double, double> IQR(std::vector<LidarPoint> &inVecLidar) 
{   
    double Q1, Q3;
    
    if(inVecLidar.size()%2 != 0)
    {
        // Index of findMedianIndex of entire data 
        int mid_index = findMedianIndex(0, inVecLidar.size()-1); 
        // cout << "Mid-Index: " << mid_index << endl;
        
        if(mid_index%2 != 0)
        {
            // Median of first half 
            Q1 = inVecLidar[findMedianIndex(0, mid_index-1)].x; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            Q3 = inVecLidar[findMedianIndex(mid_index + 1, inVecLidar.size()-1)].x;
            // cout << "Q3: " << Q3 << endl;
        }
        else
        {
            // Median of first half 
            int q1_prev = findMedianIndex(0, mid_index - 1);
            Q1 = (double)(inVecLidar[q1_prev].x + inVecLidar[q1_prev+1].x)/2; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            int q3_prev = findMedianIndex(mid_index + 1, inVecLidar.size()-1); 
            Q3 = (double)(inVecLidar[q3_prev].x + inVecLidar[q3_prev+1].x)/2;
            // cout << "Q3: " << Q3 << endl;
        }
    }
    else
    {
        int mid_index = findMedianIndex(0, inVecLidar.size()-1);
        // cout << "Mid-Index: " << mid_index << endl;
        
        if((mid_index+1)%2 != 0)
        {
            // Median of first half 
            Q1 = inVecLidar[findMedianIndex(0, mid_index)].x; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            Q3 = inVecLidar[findMedianIndex(mid_index + 1, inVecLidar.size()-1)].x;
            // cout << "Q3: " << Q3 << endl;
        }
        else
        {
            // Median of first half 
            int q1_prev = findMedianIndex(0, mid_index);
            Q1 = (double)(inVecLidar[q1_prev].x + inVecLidar[q1_prev+1].x)/2; 
            // cout << "Q1: " << Q1 << endl;
            
            // Median of second half 
            int q3_prev = findMedianIndex(mid_index + 1, inVecLidar.size()-1); 
            Q3 = (double)(inVecLidar[q3_prev].x + inVecLidar[q3_prev+1].x)/2;
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

void removeOutliersLidar(std::vector<LidarPoint> &lidarPoints)
{
    std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint p1, LidarPoint p2) -> bool{ return p1.x < p2.x; });
    
    auto minMaxBound = IQR(lidarPoints);

    double lower_bound, upper_bound;
    std::tie(lower_bound, upper_bound) = minMaxBound;

    lidarPoints.erase(std::remove_if(lidarPoints.begin(), lidarPoints.end(), [lower_bound, upper_bound](const LidarPoint& pt) { return (pt.x < lower_bound || pt.x > upper_bound); }), lidarPoints.end());
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