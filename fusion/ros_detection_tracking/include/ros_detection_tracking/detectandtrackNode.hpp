#ifndef DETECTANDTRACKNODE_H
#define DETECTANDTRACKNODE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp> 
#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"
#include <ros/ros.h>
#include "ros/package.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <termios.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <Eigen/Dense>
#include <ros/package.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/flann.h>
#include <opencv/cv.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
using namespace std;
class DetectandTrack{
        private:
        cv::Mat image_proj;
        image_transport::Publisher image_publisher;
        ros::Publisher ttc_publisher; //ttc_lidar publisher
        tf::TransformBroadcaster tr_br;
       
        

        struct initial_parameters
        {
            /* data */
            std::string camera_topic;
            std::string lidar_topic;
            cv::Mat camtocam_mat;
            cv::Mat cameraIn;
            cv::Mat RT;
            //crop lidar points
            float_t crop_minZ, crop_maxZ, crop_minX, crop_maxX, crop_Y, crop_minR;
            std::string yoloBasePath;
            std::string yoloClassesFile;
            std::string yoloModelConfiguration;
            std::string yoloModelWeights;
           
        }i_params;
        
        void detectandtrack_callback(const sensor_msgs::Image::ConstPtr &img, 
                                const sensor_msgs::PointCloud2::ConstPtr &pc);
        void initParams();
        void matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform & trans);

        public:
        DetectandTrack();
        float confThreshold = 0.2;  //置信度阈值
        float nmsThreshold = 0.4;   //极大值抑制阈值 
        bool bVis = false;
        float shrinkFactor = 0.10;
         bool first_frame;
        cv::Mat imgGray;
        vector<cv::KeyPoint> keypoints; 
        cv::Mat descriptors;
        std::vector<DataFrame> databuffer;
        vector<cv::DMatch> matches;
        string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
        string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
        string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
        map<int, int> bbBestMatches;
        BoundingBox *prevBB, *currBB;
        
    };


#endif