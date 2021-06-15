/* INCLUDES FOR THIS PROJECT */
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
#include <std_msgs/Float32.h>

#include "../../include/ros_detection_tracking/detectandtrackNode.hpp"

using namespace std;

/* MAIN PROGRAM */
void DetectandTrack::detectandtrack_callback(const sensor_msgs::Image::ConstPtr &img,
                                             const sensor_msgs::PointCloud2::ConstPtr &pc)
{
    /*1.construct data Frame and crop*/
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }
    cv::Mat raw_img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc, *cloud);

    cv::Mat visImg = raw_img.clone();
    cv::Mat overlay = visImg.clone();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
    cloudA = cloud;
    DataFrame frame;
    frame.cameraImg = raw_img;

    cropLidarPoints(cloudA, i_params.crop_minX, i_params.crop_maxX, i_params.crop_Y, i_params.crop_minZ, i_params.crop_maxZ); //crop pointcloud

    //frame.pointcloud = *cloudA;

    cout << "/*1.construct data Frame and crop*/ done" << endl;

    /*2.detection and classification*/
    detectObjects(frame.cameraImg, frame.boundingBoxes, confThreshold, nmsThreshold, i_params.yoloBasePath, i_params.yoloClassesFile,
                  i_params.yoloModelConfiguration, i_params.yoloModelWeights, bVis);

    cout << "/*2.detection and classification*/ has done" << endl;

    /*3.cluster the point with the bounding box*/

    clusterLidarWithROI(frame.boundingBoxes, cloudA, shrinkFactor, i_params.cameraIn, i_params.camtocam_mat, i_params.RT);
    cout << " /*3.cluster the point with the bounding box*/ done" << endl;

    /*4.detect keypoint*/
    cv::cvtColor(frame.cameraImg, imgGray, cv::COLOR_BGR2GRAY);
    detKeypointsShiTomasi(keypoints, imgGray, false);
    frame.keypoints = keypoints;
    cout << "/*4.detect keypoint*/ done" << endl;

    /*5.extract keypoint descriptors*/
    descKeypoints(frame.keypoints, frame.cameraImg, descriptors, "BRISK");
    frame.descriptors = descriptors;
    cout << "/*5.extract keypoint descriptors*/ done" << endl;

    //cout<<first_frame<<endl;
    if (first_frame)
    {
        databuffer.push_back(frame);
        first_frame = false;
        cout << "first frame done" << endl;
    }
    else
    {
        if (databuffer[0].boundingBoxes[0].lidarPoints.size() > 0)
        {
            cout << "databuffer size:" << databuffer[0].boundingBoxes[0].lidarPoints.size() << endl;
        }

        /*6.match keypoints*/
        databuffer.push_back(frame);
        matchDescriptors(databuffer[0].keypoints, databuffer[1].keypoints,
                         databuffer[0].descriptors, databuffer[1].descriptors,
                         matches, descriptorType, matcherType, selectorType);
        databuffer[1].kptMatches = matches;
        cout << "/*6.match keypoints*/ done" << endl;

        /*7.track object*/
        matchBoundingBoxes(matches, bbBestMatches, databuffer[0], databuffer[1]);
        databuffer[1].bbMatches = bbBestMatches;
        cout << "/*7.track object*/" << endl;
        //cout<<int(databuffer.size())<<endl;

        if (databuffer[1].bbMatches.size() > 0)
        {

            /*8.computer TTC*/
            for (auto it1 = databuffer[1].bbMatches.begin(); it1 != databuffer[1].bbMatches.end(); ++it1)
            {
                if (databuffer[1].boundingBoxes.size() > 0 && databuffer[0].boundingBoxes.size() > 0)
                {

                    // find bounding boxes associates with current match

                    for (auto it2 = databuffer[1].boundingBoxes.begin(); it2 != databuffer[1].boundingBoxes.end(); ++it2)
                    {
                        if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            currBB = &(*it2);
                        }
                    }

                    for (auto it2 = databuffer[0].boundingBoxes.begin(); it2 != databuffer[0].boundingBoxes.end(); ++it2)
                    {
                        if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            prevBB = &(*it2);
                        }
                    }
                }
                else
                {
                    BoundingBox *box, *box1;
                    box->boxID = -1;
                    box1->boxID = -1;
                    currBB = box;
                    prevBB = box1;
                }

                // cout<<double(currBB->lidarPoints.size())<<"and"<<double(prevBB->lidarPoints.size())<<endl;
                // if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                if (currBB->boxID != -1 && prevBB->boxID != -1 && prevBB->lidarPoints.size() > 0 && currBB->lidarPoints.size() > 0)
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar;
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, 10, ttcLidar, true);
                    std_msgs::Float32 ttc_lidar;
                    ttc_lidar.data = ttcLidar;
                    ttc_publisher.publish(ttc_lidar);
                    cout << "/*8.computer TTC*/ done" << endl;
                }
            }
        }

        databuffer[0] = databuffer[1];
        databuffer.pop_back();
        cout << databuffer.size() << endl;
        cout << "frame pairs process done" << endl;
    }
}

void DetectandTrack::initParams()
{
    std::string pkg_loc = ros::package::getPath("ros_detection_tracking");
    std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");
    infile >> i_params.camera_topic;
    infile >> i_params.lidar_topic;
    infile >> i_params.crop_minZ;
    infile >> i_params.crop_maxZ;
    infile >> i_params.crop_minX;
    infile >> i_params.crop_maxX;
    infile >> i_params.crop_Y;
    infile >> i_params.crop_minR;
    //std::cout<<i_params.camera_topic<<std::endl;
    double_t camtocam[12];
    double_t cameraIn[16];
    double_t RT[16];
    for (int i = 0; i < 16; i++)
    {
        infile >> camtocam[i];
    }
    cv::Mat(4, 4, CV_64F, &camtocam).copyTo(i_params.camtocam_mat); //cameratocamera params

    for (int i = 0; i < 12; i++)
    {
        infile >> cameraIn[i];
    }
    cv::Mat(3, 4, CV_64F, &cameraIn).copyTo(i_params.cameraIn); //cameraIn params

    for (int i = 0; i < 16; i++)
    {
        infile >> RT[i];
    }
    cv::Mat(4, 4, CV_64F, &RT).copyTo(i_params.RT); //lidar to camera params
    //std::cout<<i_params.RT<<std::endl;

    //detect_config file
    std::string pkg_loc1 = ros::package::getPath("ros_detection_tracking");
    std::ifstream infile1(pkg_loc1 + "/cfg/detect_config.txt");
    infile1 >> i_params.yoloBasePath;
    infile1 >> i_params.yoloClassesFile;
    infile1 >> i_params.yoloModelConfiguration;
    infile1 >> i_params.yoloModelWeights;
}

DetectandTrack::DetectandTrack()
{
    ros::NodeHandle nh("~");
    initParams();
    first_frame = true;

    //lidar and camera time synchronize
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, i_params.camera_topic, 5);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, i_params.lidar_topic, 5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, pcl_sub);
    sync.registerCallback(boost::bind(&DetectandTrack::detectandtrack_callback, this, _1, _2));
    ttc_publisher = nh.advertise<std_msgs::Float32>("/ttc_lidar", 1);
    ros::spin();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "detect_track_ros");
    DetectandTrack DT;
    return 0;
}
