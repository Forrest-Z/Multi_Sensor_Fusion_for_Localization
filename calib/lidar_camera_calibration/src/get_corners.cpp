#include "lidar_camera_calibration/get_corners.h"

static int cnt_corner = 0;
static std::vector<std::vector<cv::Point>> stored_corners;

bool getCorners(cv::Mat img, pcl::PointCloud<pcl::PointXYZ> scan, cv::Mat P, int num_of_markers, int MAX_ITERS)
{
    std::cout << "当前是第: " << cnt_corner << " 次迭代找角点" << std::endl;

    /*Masking happens here */
    cv::Mat edge_mask = cv::Mat::zeros(img.size(), CV_8UC1);
    //edge_mask(cv::Rect(520, 205, 300, 250))=1;
    edge_mask(cv::Rect(0, 0, img.cols, img.rows)) = 1;
    img.copyTo(edge_mask, edge_mask);
    //pcl::io::savePCDFileASCII ("/home/vishnu/final1.pcd", scan.point_cloud);
    img = edge_mask;
    //cv:imwrite("/home/vishnu/marker.png", edge_mask);

    pcl::PointCloud<pcl::PointXYZ> pc = scan;
    //scan = Velodyne::Velodyne(filtered_pc);

    cv::Rect frame(0, 0, img.cols, img.rows);

    //pcl::io::savePCDFileASCII("/home/vishnu/final2.pcd", scan.point_cloud);

    cv::Mat image_edge_laser = project(P, frame, scan, NULL);
    cv::threshold(image_edge_laser, image_edge_laser, 10, 255, 0);

    cv::Mat combined_rgb_laser;
    std::vector<cv::Mat> rgb_laser_channels;

    rgb_laser_channels.push_back(image_edge_laser);
    rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
    rgb_laser_channels.push_back(img);

    cv::merge(rgb_laser_channels, combined_rgb_laser);
    /*cv::namedWindow("combined", cv::WINDOW_NORMAL); 
	cv::imshow("combined", combined_rgb_laser);
	cv::waitKey(5);
	*/

    /* 用一个map存储像素点和三维点对应关系 */
    std::map<std::pair<int, int>, std::vector<float>> c2D_to_3D;
    std::vector<float> point_3D;
    for (auto pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
        // 这里的pc已经大致投影到了相机坐标系下面，因此先把相机背面(z<0)的点云删除
        if (pt->z < 0)
        {
            continue;
        }
        // 相机前面的点云投影在相机的像素平面上
        cv::Point xy = project(*pt, P);
        if (xy.inside(frame))
        {
            point_3D.clear();
            point_3D.push_back(pt->x);
            point_3D.push_back(pt->y);
            point_3D.push_back(pt->z);
            c2D_to_3D[std::pair<int, int>(xy.x, xy.y)] = point_3D;
        }
    }

    /* print the correspondences */
    /*for(std::map<std::pair<int, int>, std::vector<float> >::iterator it=c2D_to_3D.begin(); it!=c2D_to_3D.end(); ++it)
	{
		std::cout << it->first.first << "," << it->first.second << " --> " << it->second[0] << "," <<it->second[1] << "," <<it->second[2] << "\n";
	}*/

    /* get region of interest */
    const int QUADS = num_of_markers;
    std::vector<int> LINE_SEGMENTS(QUADS, 4); //assuming each has 4 edges and 4 corners
    pcl::PointCloud<pcl::PointXYZ>::Ptr board_corners(new pcl::PointCloud<pcl::PointXYZ>);
    // marker是最终的拟合出来的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr marker(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> c_3D;
    std::vector<cv::Point2f> c_2D;

    cv::namedWindow("cloud", cv::WINDOW_NORMAL);
    cv::namedWindow("拟合", cv::WINDOW_NORMAL);
    //cv::namedWindow("combined", cv::WINDOW_NORMAL);

    std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
    std::ofstream outfile(pkg_loc + "/log/points.txt", std::ios_base::trunc);
    outfile << QUADS * 4 << "\n";

    for (int q = 0; q < QUADS; q++)
    {
        std::cout << "---------下一个marker--------\n";
        std::vector<Eigen::VectorXf> line_model;
        for (int i = 0; i < LINE_SEGMENTS[q]; i++)
        {
            cv::Point _point_;
            std::vector<cv::Point> polygon;
            int collected;

            // get markings in the first iteration only
            if (cnt_corner == 0)
            {
                polygon.clear();
                collected = 0;
                while (collected != LINE_SEGMENTS[q])
                {
                    cv::setMouseCallback("cloud", onMouse, &_point_);
                    cv::imshow("cloud", image_edge_laser);
                    cv::waitKey(0);
                    ++collected;
                    //std::cout << _point_.x << " " << _point_.y << "\n";
                    polygon.push_back(_point_);
                }
                stored_corners.push_back(polygon);
            }

            polygon = stored_corners[4 * q + i];

            cv::Mat polygon_image = cv::Mat::zeros(image_edge_laser.size(), CV_8UC1);

            rgb_laser_channels.clear();
            rgb_laser_channels.push_back(image_edge_laser);
            rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
            rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
            cv::merge(rgb_laser_channels, combined_rgb_laser);

            for (int j = 0; j < 4; j++)
            {
                cv::line(combined_rgb_laser, polygon[j], polygon[(j + 1) % 4], cv::Scalar(0, 255, 0));
            }

            // initialize PointClouds
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

            for (auto it = c2D_to_3D.begin(); it != c2D_to_3D.end(); ++it)
            {
                if (cv::pointPolygonTest(cv::Mat(polygon), cv::Point(it->first.first, it->first.second), true) > 0)
                {
                    cloud->push_back(pcl::PointXYZ(it->second[0], it->second[1], it->second[2]));
                    rectangle(combined_rgb_laser, cv::Point(it->first.first, it->first.second), cv::Point(it->first.first, it->first.second), cv::Scalar(0, 0, 255), 3, 8, 0); // RED point
                }
            }

            if (cloud->size() < 2)
            {
                return false;
            }

            cv::imshow("多边形", combined_rgb_laser);
            cv::waitKey(4);

            //pcl::io::savePCDFileASCII("/home/lixiang/line_cloud.pcd", *cloud);

            std::vector<int> inliers;
            Eigen::VectorXf model_coefficients;

            // created RandomSampleConsensus object and compute the appropriated model
            // 拿ransac拟合直线
            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
            ransac.setDistanceThreshold(0.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
            ransac.getModelCoefficients(model_coefficients);
            line_model.push_back(model_coefficients);

            std::cout << "当前直线参数:\n"
                      << model_coefficients << std::endl;
            // copies all inliers of the model computed to another PointCloud
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
            //pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_line_cloud.pcd", *final);
            *marker += *final;
        }

        /* calculate approximate intersection of lines */

        Eigen::Vector4f p1, p2, p_intersect;
        pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < LINE_SEGMENTS[q]; i++)
        {
            pcl::lineToLineSegment(line_model[i], line_model[(i + 1) % LINE_SEGMENTS[q]], p1, p2);
            for (int j = 0; j < 4; j++)
            {
                p_intersect(j) = (p1(j) + p2(j)) / 2.0;
            }
            c_3D.push_back(cv::Point3f(p_intersect(0), p_intersect(1), p_intersect(2)));
            corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
            std::cout << "Point of intersection is approximately: \n"
                      << p_intersect << "\n";
            //std::cout << "Distance between the lines: " << (p1 - p2).squaredNorm () << "\n";
            std::cout << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";
            outfile << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";
        }

        *board_corners += *corners;

        std::cout << "Distance between the corners:\n";
        for (int i = 0; i < 4; i++)
        {
            std::cout << sqrt(
                             pow(c_3D[4 * q + i].x - c_3D[4 * q + (i + 1) % 4].x, 2) + pow(c_3D[4 * q + i].y - c_3D[4 * q + (i + 1) % 4].y, 2) + pow(c_3D[4 * q + i].z - c_3D[4 * q + (i + 1) % 4].z, 2))
                      << std::endl;
        }
    }
    outfile.close();

    cnt_corner++;
    if (cnt_corner == MAX_ITERS)
    {
        ros::shutdown();
    }
    return true;
    // 保存点云
    // pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_marker.pcd", *marker);
    // pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_corners.pcd", *board_corners);
}

// 根据外参矩阵P把点云投影到相机坐标系的归一化平面下
cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix)
{
    //cv::Point2f xy = projectf(pt, projection_matrix);
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f;

    cv::Mat pt_2D = projection_matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;
    return cv::Point(x, y);
}

cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> *visible_points)
{
    cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {

        // behind the camera
        if (pt->z < 0)
        {
            continue;
        }

        //float intensity = pt->intensity;
        cv::Point xy = project(*pt, projection_matrix);
        if (xy.inside(frame))
        {
            if (visible_points != NULL)
            {
                visible_points->push_back(*pt);
            }

            //cv::circle(plane, xy, 3, intensity, -1);
            //plane.at<float>(xy) = intensity;
            plane.at<float>(xy) = 250;
        }
    }
    cv::Mat plane_gray;
    cv::normalize(plane, plane_gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::dilate(plane_gray, plane_gray, cv::Mat());

    return plane_gray;
}

// 鼠标回调
void onMouse(int event, int x, int y, int f, void *g)
{

    cv::Point *P = static_cast<cv::Point *>(g);
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:

        P->x = x;
        P->y = y;
        break;
    case CV_EVENT_LBUTTONUP:
        P->x = x;
        P->y = y;
        //std::cout << P->x << " " << P->y << "\n";
        break;

    default:
        break;
    }
}