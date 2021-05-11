#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using std::cout;
using std::endl;

int main()
{
    //旋转矩阵->欧拉角
    //cam1
    Eigen::Matrix4d lidar_to_cam1_T;
    lidar_to_cam1_T << 0.718493, -0.695488, -0.00798845, 0.0543754,
        0.00973158, 0.0215364, -0.999721, -0.0286391,
        0.695466, 0.718215, 0.0222419, -0.0739404,
        0, 0, 0, 1;
    Eigen::Matrix3d lidar_to_cam1_rot = lidar_to_cam1_T.block(0, 0, 3, 3);
    cout << "xyzrpy1:" << lidar_to_cam1_T.block(0, 3, 3, 1).transpose() << " "
         << lidar_to_cam1_rot.eulerAngles(2, 1, 0).transpose() << endl;
    cout << "逆变换xyzrpy1:" << (-lidar_to_cam1_rot.transpose() * lidar_to_cam1_T.block(0, 3, 3, 1)).transpose() << " "
         << lidar_to_cam1_rot.inverse().eulerAngles(2, 1, 0).transpose() << endl;

    //    cam2
    Eigen::Matrix4d lidar_to_cam2_T;
    lidar_to_cam2_T << 0.678062, 0.734785, 0.0179658, 0.0503224,
        0.0386084, -0.0111973, -0.999192, -0.0382712,
        -0.73399, 0.678207, -0.0359614, -0.0744093,
        0, 0, 0, 1;
    Eigen::Matrix3d lidar_to_cam2_rot = lidar_to_cam2_T.block(0, 0, 3, 3);
    cout << "xyzrpy2:" << lidar_to_cam2_T.block(0, 3, 3, 1).transpose() << " "
         << lidar_to_cam2_rot.eulerAngles(2, 1, 0).transpose() << endl;
    cout << "逆变换xyzrpy2:" << (-lidar_to_cam2_rot.transpose() * lidar_to_cam2_T.block(0, 3, 3, 1)).transpose() << " "
         << lidar_to_cam2_rot.inverse().eulerAngles(2, 1, 0).transpose() << endl;

    //cam3
    Eigen::Matrix4d lidar_to_cam3_T;
    lidar_to_cam3_T << -0.723187, 0.690105, 0.0274755, 0.0532929,
        -0.0107868, 0.0284911, -0.999536, -0.0352281,
        -0.690568, -0.723148, -0.0131604, -0.0740557,
        0, 0, 0, 1;
    Eigen::Matrix3d lidar_to_cam3_rot = lidar_to_cam3_T.block(0, 0, 3, 3);
    cout << "xyzrpy3:" << lidar_to_cam3_T.block(0, 3, 3, 1).transpose() << " "
         << lidar_to_cam3_rot.eulerAngles(2, 1, 0).transpose() << endl;
    cout << "逆变换xyzrpy3:" << (-lidar_to_cam3_rot.transpose() * lidar_to_cam3_T.block(0, 3, 3, 1)).transpose() << " "
         << lidar_to_cam3_rot.inverse().eulerAngles(2, 1, 0).transpose() << endl;

    //cam4
    Eigen::Matrix4d lidar_to_cam4_T;
    lidar_to_cam4_T << -0.688299, -0.725344, 0.0110158, 0.0643419,
        -0.0311459, 0.0143772, -0.999411, -0.0527804,
        0.724758, -0.688237, -0.0324873, -0.0808813,
        0, 0, 0, 1;
    Eigen::Matrix3d lidar_to_cam4_rot = lidar_to_cam4_T.block(0, 0, 3, 3);
    cout << "xyzrpy4:" << lidar_to_cam4_T.block(0, 3, 3, 1).transpose() << " "
         << lidar_to_cam4_rot.eulerAngles(2, 1, 0).transpose() << endl;
    cout << "逆变换xyzrpy4:" << (-lidar_to_cam4_rot.transpose() * lidar_to_cam4_T.block(0, 3, 3, 1)).transpose() << " "
         << lidar_to_cam4_rot.inverse().eulerAngles(2, 1, 0).transpose() << endl;

    return 0;
}
