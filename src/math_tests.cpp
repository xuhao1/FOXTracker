#include <iostream>
#include <Eigen/Dense>
#include <utils.h>


int math_tests() {
    auto rot = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX());
    auto mat = rot.toRotationMatrix();
    std::cout << "ROTM\n" << mat << std::endl;

    Eigen::Vector3d ea = mat.eulerAngles(2, 1, 0);
    std::cout << "to Euler angles:" << ea.transpose() << std::endl;

    Eigen::Quaterniond q(rot);
    auto eul = quat2eulers(q, false);

    std::cout << "quat2eulers:" << eul.transpose() << std::endl;

    return 0;
}
