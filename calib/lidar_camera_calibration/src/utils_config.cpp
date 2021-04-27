#include <lidar_camera_calibration/utils_config.h>

/* 读取参数部分 */
void print_config(const config_data &config)
{
    std::cout << "***********当前参数***********" << std::endl;
    std::cout << "图像尺寸: " << config.s.width << " x " << config.s.height << std::endl;
    std::cout << "点云范围(相机坐标系下的xyz):\n ";
    for (int i = 0; i < 3; i++)
    {
        std::cout << config.xyz_[i].first << " - " << config.xyz_[i].second << std::endl;
    }
    std::cout << "标签个数: " << config.num_of_markers << std::endl;
    std::cout << "激光强度阈值(0-1): " << config.intensity_thresh << std::endl;
    std::cout << "内参矩阵:\n"
              << config.P << std::endl;
    std::cout << "迭代次数: " << config.MAX_ITERS << std::endl;
    std::cout << "初始旋转:\n"
              << config.initialRot[0] << " "
              << config.initialRot[1] << " "
              << config.initialRot[2] << "\n";
    std::cout << "初始平移:\n"
              << config.initialTra[0] << " "
              << config.initialTra[1] << " "
              << config.initialTra[2] << "\n";
}

void read_config(config_data &config)
{
    std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
    std::ifstream infile(pkg_loc + "/conf/config_file.txt");
    float left_limit = 0.0, right_limit = 0.0;
    float angle;
    float dist;
    // 读取config_file.txt参数
    infile >> config.s.width >> config.s.height;
    for (int i = 0; i < 3; i++)
    {
        infile >> left_limit >> right_limit;
        config.xyz_.push_back(std::pair<float, float>(left_limit, right_limit));
    }
    infile >> config.intensity_thresh >> config.num_of_markers;
    float p[12];
    for (int i = 0; i < 12; i++)
    {
        infile >> p[i];
    }
    cv::Mat(3, 4, CV_32FC1, &p).copyTo(config.P);
    infile >> config.MAX_ITERS;
    for (int i = 0; i < 3; i++)
    {
        infile >> angle;
        config.initialRot.push_back(angle);
    }
    for (int i = 0; i < 3; i++)
    {
        infile >> dist;
        config.initialTra.push_back(dist);
    }
    infile.close();
}
