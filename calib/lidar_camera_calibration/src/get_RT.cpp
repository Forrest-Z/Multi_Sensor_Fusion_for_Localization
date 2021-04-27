#include "lidar_camera_calibration/get_RT.h"

static std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
static int cnt_RT = 0;
static Eigen::Vector3d translation_sum;
static Eigen::Quaterniond rotation_sum;
static Eigen::Matrix3d rotation_avg_by_mult;
static float rmse_avg;

Eigen::Quaterniond addQ(Eigen::Quaterniond a, Eigen::Quaterniond b)
{
    Eigen::Quaterniond retval;
    if (a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w() < 0.0)
    {
        b.x() = -b.x();
        b.y() = -b.y();
        b.z() = -b.z();
        b.w() = -b.w();
    }
    retval.x() = a.x() + b.x();
    retval.y() = a.y() + b.y();
    retval.z() = a.z() + b.z();
    retval.w() = a.w() + b.w();
    return retval;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> readArray()
{
    std::ifstream infile(pkg_loc + "/log/points.txt");
    int num_points = 0;

    infile >> num_points;

    ROS_ASSERT(num_points > 0);

    Eigen::MatrixXd lidar(3, num_points), camera(3, num_points);

    std::cout << "Num points is:" << num_points << std::endl;

    for (int i = 0; i < num_points; i++)
    {
        infile >> lidar(0, i) >> lidar(1, i) >> lidar(2, i);
    }
    for (int i = 0; i < num_points; i++)
    {
        infile >> camera(0, i) >> camera(1, i) >> camera(2, i);
    }
    infile.close();

    // camera values are stored in variable 'lidar' and vice-versa
    // need to change this
    return std::pair<Eigen::MatrixXd, Eigen::MatrixXd>(lidar, camera);
    //return std::pair<MatrixXd, MatrixXd>(camera, lidar);
}

// calculates rotation and translation that transforms points in the lidar frame to the camera frame
// 对传进来的lidar中的corner点和camera中的corner点，计算 lidar->camera 的旋转和平移矩阵
void calc_RT(Eigen::MatrixXd lidar, Eigen::MatrixXd camera, int MAX_ITERS, Eigen::Matrix3d lidarToCamera)
{
    // 第一次迭代的时候，清空输出文件
    if (cnt_RT == 0)
    {
        std::ofstream clean_file(pkg_loc + "/log/avg_values.txt", std::ios_base::trunc);
        clean_file.close();

        translation_sum << 0.0, 0.0, 0.0;
        rotation_sum = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
        rotation_avg_by_mult << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
        rmse_avg = 0.0;
    }
    int num_points = lidar.cols();

    std::cout << "参与ICP匹配的点数: " << num_points << std::endl;
    Eigen::Vector3d mu_lidar, mu_camera;

    mu_lidar << 0.0, 0.0, 0.0;
    mu_camera << 0.0, 0.0, 0.0;

    for (int i = 0; i < num_points; i++)
    {
        mu_lidar(0) += lidar(0, i);
        mu_lidar(1) += lidar(1, i);
        mu_lidar(2) += lidar(2, i);
    }
    mu_lidar = mu_lidar / num_points;
    for (int i = 0; i < num_points; i++)
    {
        mu_camera(0) += camera(0, i);
        mu_camera(1) += camera(1, i);
        mu_camera(2) += camera(2, i);
    }
    mu_camera = mu_camera / num_points;

    if (cnt_RT == 0)
    {
        std::cout << "mu_lidar:\n"
                  << mu_lidar << std::endl;
        std::cout << "mu_camera:\n"
                  << mu_camera << std::endl;
    }

    Eigen::MatrixXd lidar_centered = lidar.colwise() - mu_lidar;
    Eigen::MatrixXd camera_centered = camera.colwise() - mu_camera;
    if (cnt_RT == 0)
    {
        std::cout << "雷达去中心点:\n"
                  << lidar_centered << std::endl;
        std::cout << "相机去中心点:\n"
                  << camera_centered << std::endl;
    }

    Eigen::Matrix3d cov = camera_centered * lidar_centered.transpose();
    // std::cout << "cov:\n"
    //           << cov << std::endl;

    // SVD分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation;
    rotation = svd.matrixU() * svd.matrixV().transpose();
    if (rotation.determinant() < 0)
    {
        Eigen::Vector3d diag_correct;
        diag_correct << 1.0, 1.0, -1.0;
        rotation = svd.matrixU() * diag_correct.asDiagonal() * svd.matrixV().transpose();
    }

    Eigen::Vector3d translation = mu_camera - rotation * mu_lidar;

    // averaging translation and rotation
    translation_sum += translation;
    Eigen::Quaterniond temp_q(rotation);
    rotation_sum = addQ(rotation_sum, temp_q);

    // averaging rotations by multiplication
    rotation_avg_by_mult = rotation_avg_by_mult.pow(1.0 * cnt_RT / (cnt_RT + 1)) * rotation.pow(1.0 / (cnt_RT + 1));

    std::cout << "匹配点之间的旋转矩阵: \n"
              << rotation << std::endl;
    std::cout << "匹配点之间的旋转欧拉角(zyx):\n"
              << rotation.eulerAngles(2, 1, 0) << std::endl;
    std::cout << "匹配点之间的平移向量:\n"
              << translation << std::endl;

    Eigen::MatrixXd eltwise_error = (camera - ((rotation * lidar).colwise() + translation)).array().square().colwise().sum();
    double error = sqrt(eltwise_error.sum() / num_points);
    std::cout << "均方根误差: " << error << std::endl;

    rmse_avg = rmse_avg + error;

    Eigen::Matrix4d T;
    T.setIdentity(4, 4);
    T.topLeftCorner(3, 3) = rotation;
    T.col(3).head(3) = translation;

    std::cout << "匹配点之间的空间变换矩阵T:\n"
              << T << std::endl;

    cnt_RT++;
    //if(cnt_RT == MAX_ITERS)
    // 注意不能让cnt_RT == 0进来，里面要算除法
    if (cnt_RT != 0)
    {
        std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);

        std::cout << "--------------------------------------------------------------------\n";
        std::cout << "经过 " << cnt_RT << " 次找R变换矩阵迭代\n";
        std::cout << "--------------------------------------------------------------------\n";
        std::cout << "Average translation is:\n"
                  << translation_sum / cnt_RT << "\n";
        log_avg_values << cnt_RT << "\n";
        log_avg_values << translation_sum / cnt_RT << "\n";

        rotation_sum.x() = rotation_sum.x() / cnt_RT;
        rotation_sum.y() = rotation_sum.y() / cnt_RT;
        rotation_sum.z() = rotation_sum.z() / cnt_RT;
        rotation_sum.w() = rotation_sum.w() / cnt_RT;
        double mag = sqrt(rotation_sum.x() * rotation_sum.x() +
                          rotation_sum.y() * rotation_sum.y() +
                          rotation_sum.z() * rotation_sum.z() +
                          rotation_sum.w() * rotation_sum.w());
        rotation_sum.x() = rotation_sum.x() / mag;
        rotation_sum.y() = rotation_sum.y() / mag;
        rotation_sum.z() = rotation_sum.z() / mag;
        rotation_sum.w() = rotation_sum.w() / mag;

        Eigen::Matrix3d rotation_avg = rotation_sum.toRotationMatrix();
        std::cout << "Average rotation is:\n"
                  << rotation_avg << std::endl;
        Eigen::Matrix3d final_rotation = rotation_avg * lidarToCamera;
        Eigen::Vector3d final_angles = final_rotation.eulerAngles(2, 1, 0);
        /* 
        std::cout << "Average rotation by multiplication is:\n"
                  << rotation_avg_by_mult << std::endl;
        std::cout << rotation_avg(0, 0) << " " << rotation_avg(0, 1) << " " << rotation_avg(0, 2) << "\n"
                  << rotation_avg(1, 0) << " " << rotation_avg(1, 1) << " " << rotation_avg(1, 2) << "\n"
                  << rotation_avg(2, 0) << " " << rotation_avg(2, 1) << " " << rotation_avg(2, 2) << "\n";
        */
        log_avg_values << std::fixed << std::setprecision(8)
                       << rotation_avg(0, 0) << " " << rotation_avg(0, 1) << " " << rotation_avg(0, 2) << "\n"
                       << rotation_avg(1, 0) << " " << rotation_avg(1, 1) << " " << rotation_avg(1, 2) << "\n"
                       << rotation_avg(2, 0) << " " << rotation_avg(2, 1) << " " << rotation_avg(2, 2) << "\n";

        Eigen::Matrix4d T;
        T.setIdentity(4, 4);
        T.topLeftCorner(3, 3) = rotation_avg;
        T.col(3).head(3) = translation_sum / cnt_RT;
        std::cout << "Average transformation is:\n"
                  << T << "\n";
        std::cout << "Final rotation is:\n"
                  << final_rotation << "\n";
        std::cout << "Final ypr is:\n"
                  << final_angles << "\n";
        std::cout << "Average RMSE is: " << rmse_avg * 1.0 / cnt_RT << "\n";

        Eigen::MatrixXd eltwise_error_temp = (camera - ((rotation_avg * lidar).colwise() + (translation_sum / cnt_RT))).array().square().colwise().sum();
        double error_temp = sqrt(eltwise_error_temp.sum() / num_points);

        std::cout << "RMSE on average transformation is: " << error_temp << std::endl;
        log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";
    }
}

void readArucoPose(std::vector<float> marker_info, int num_of_marker_in_config)
{
    std::vector<Eigen::Matrix4d> marker_pose;

    ROS_ASSERT(marker_info.size() / 7 == num_of_marker_in_config);

    int j = 0;
    for (int i = 0; i < marker_info.size() / 7; i++)
    {

        //std::cout << "In readArucoPose(): " << std::endl;

        Eigen::Vector3d trans, rot;
        int marker_id = marker_info[j++];
        trans(0) = marker_info[j++];
        trans(1) = marker_info[j++];
        trans(2) = marker_info[j++];
        rot(0) = marker_info[j++];
        rot(1) = marker_info[j++];
        rot(2) = marker_info[j++];

        //std::cout << "\n" << "Marker id:" << marker_id << "\n" << trans << "\n" << rot << std::endl;

        Eigen::Transform<double, 3, Eigen::Affine> aa;
        aa = Eigen::AngleAxis<double>(rot.norm(), rot / rot.norm());

        Eigen::Matrix4d g;
        g.setIdentity(4, 4);
        //std::cout << "Rot matrix is: \n" << aa*g << std::endl;
        g = aa * g;

        Eigen::Matrix4d T;
        T.setIdentity(4, 4);
        T.topLeftCorner(3, 3) = g.topLeftCorner(3, 3); //.transpose();
        T.col(3).head(3) = trans;

        marker_pose.push_back(T);

        //std::cout << "transformation matrix is: \n" << T << std::endl;
    }

    //std::vector<std::vector<std::pair<float, float> > > marker_coordinates;
    std::ifstream infile(pkg_loc + "/conf/marker_coordinates.txt");
    std::ofstream outfile(pkg_loc + "/log/points.txt", std::ios_base::app);

    int num_of_markers;
    infile >> num_of_markers;

    for (int i = 0; i < num_of_markers; i++)
    {
        float temp;
        std::vector<float> board;
        //std::vector<std::pair<float, float> > corner_points;
        for (int j = 0; j < 5; j++)
        {
            infile >> temp;
            board.push_back(temp / 100.0);
        }
        float la, ba;
        la = board[4] / 2 + board[2];
        ba = board[4] / 2 + board[3];

        /*corner_points.push_back(std::make_pair(ba, 		   board[0]-la));
    	corner_points.push_back(std::make_pair(ba-board[1], board[0]-la));
    	corner_points.push_back(std::make_pair(ba-board[1], -la 		  ));
    	corner_points.push_back(std::make_pair(ba, 		   -la 		  ));*/

        Eigen::Matrix4d points_board;
        points_board << ba, 0, board[0] - la, 1,
            ba - board[1], 0, board[0] - la, 1,
            ba - board[1], 0, -la, 1,
            ba, 0, -la, 1;

        /*std::cout << "Points in before transform: \n" << points_board << std::endl;*/

        points_board = marker_pose[i] * (points_board.transpose());

        /*std::cout << "Board number: " << i+1 << "\n";
    	std::cout << "P1: " << ba << " " << board[0]-la << "\n";
    	std::cout << "P2: " << ba-board[1] << " " << board[0]-la << "\n";
    	std::cout << "P3: " << ba-board[1] << " " << -la << "\n";
    	std::cout << "P4: " << ba << " " << -la << "\n\n";

    	std::cout << "Points in camera frame: \n" << points_board << std::endl;*/

        //marker_coordinates.push_back(corner_points);

        for (int k = 0; k < 4; k++)
        {
            outfile << points_board(0, k) << " " << points_board(1, k) << " " << points_board(2, k) << "\n";
        }
    }
    outfile.close();
    infile.close();
}

void find_transformation(std::vector<float> marker_info, int num_of_marker_in_config, int MAX_ITERS, Eigen::Matrix3d lidarToCamera)
{
    readArucoPose(marker_info, num_of_marker_in_config);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> point_clouds = readArray();
    calc_RT(point_clouds.first, point_clouds.second, MAX_ITERS, lidarToCamera);
}