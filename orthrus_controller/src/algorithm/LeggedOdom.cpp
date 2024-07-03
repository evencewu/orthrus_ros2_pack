#include "orthrus_controller/algorithm/LeggedOdom.hpp"

namespace orthrus_controller
{
    void LeggedOdom::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
        OdomFilterInit(LeggedOdom::KF);
    }

    void LeggedOdom::Calibration(rclcpp::Time time, rclcpp::Duration duration)
    {
    }

    void LeggedOdom::Update(rclcpp::Time time, rclcpp::Duration duration)
    {
        auto *odom = &orthrus_interfaces_->odom_state;
        auto *robot = &orthrus_interfaces_->robot_state;

        odom->imu = robot->body_imu;
        odom->euler = Quaternion2Euler(odom->imu.orientation);

        odom->dt = (double)(duration.nanoseconds()) / 1000000000;

        double dt = odom->dt;

        OdomFilterUpdate();
    }

    Eigen::Vector3d LeggedOdom::Quaternion2Euler(Eigen::Quaterniond quat)
    {
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
        return euler_angles;
    }

    void LeggedOdom::OdomFilterInit(int filter_type)
    {
        filter_type_ = filter_type;

        if (filter_type_ == LeggedOdom::EKF)
        {
        }
        else if (filter_type_ == LeggedOdom::KF)
        {
            kf.x_last << 0, 0, 0, 0, 0, 0;

            kf.P_last.setIdentity(); // 将 P 初始化为单位矩阵
            kf.P_last *= 0.1;

            kf.H << 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 1;

            kf.R << 0.1, 0,
                0, 0.1;
        }
    }

    void LeggedOdom::OdomFilterUpdate()
    {
        auto *odom = &orthrus_interfaces_->odom_state;
        double dt = odom->dt;

        // imu Vxy数据对齐odom轴
        orthrus_interfaces_->odom_state.imu_acceleration = orthrus_interfaces_->odom_state.imu.orientation * orthrus_interfaces_->odom_state.imu.linear_acceleration + orthrus_interfaces_->odom_state.gravity;

        //--
        double a_x = odom->imu_acceleration[0];
        double a_y = odom->imu_acceleration[1];

        // orthrus_interfaces_->odom_state.imu_velocity += orthrus_interfaces_->odom_state.imu_acceleration * dt;

        // odom->imu.orientation;

        if (filter_type_ == LeggedOdom::EKF)
        {
        }
        else if (filter_type_ == LeggedOdom::KF)
        {

            kf.x_last = kf.x;
            kf.P_last = kf.P;

            kf.x << 0, 0, 0, 0, a_x, a_y;
            kf.F << 1, 0, dt, 0, dt * dt / 2, 0,
                0, 1, 0, dt, 0, dt * dt / 2,
                0, 0, 1, 0, dt, 0,
                0, 0, 0, 1, 0, dt,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
            kf.z << a_x, a_y;

            kf.Q << kf.d_a_x * std::pow(dt, 4) / 4, 0, kf.d_a_x * std::pow(dt, 3) / 2, 0, kf.d_a_x * std::pow(dt, 2) / 2, 0,
                0, kf.d_a_y * std::pow(dt, 4) / 4, 0, kf.d_a_y * std::pow(dt, 3) / 2, 0, kf.d_a_y * std::pow(dt, 2) / 2,
                kf.d_a_x * std::pow(dt, 3) / 2, 0, kf.d_a_x * std::pow(dt, 2), 0, dt, 0,
                0, kf.d_a_y * std::pow(dt, 3) / 2, 0, kf.d_a_y * std::pow(dt, 2), 0, dt,
                kf.d_a_x * std::pow(dt, 2) / 2, 0, dt, 0, kf.d_a_x, 0,
                0, kf.d_a_y * std::pow(dt, 2) / 2, 0, dt, 0, kf.d_a_y;

            kf.x_priori = kf.F * kf.x_last;
            // kf.x_priori = kf.F * kf.x + kf.G * kf.u;

            kf.P_priori = kf.F * kf.P_last * kf.F.transpose() + kf.Q;

            kf.K = kf.P_priori * kf.H.transpose() * (kf.H * kf.P_priori * kf.H.transpose() + kf.R).inverse();

            kf.x = kf.x_priori + kf.K * (kf.z - kf.H * kf.x_priori);

            kf.P = (kf.I - kf.K * kf.H) * kf.P_last.inverse();

            orthrus_interfaces_->odom_state.imu_velocity[0] = kf.x[0];
            orthrus_interfaces_->odom_state.imu_velocity[1] = kf.x[1];
            orthrus_interfaces_->odom_state.imu_velocity[2] = 0;
        }
    }

    std::stringstream LeggedOdom::OdomFilterLog()
    {
        std::stringstream ss;

        ss << "kf.x_priori\n"
           << kf.x_priori << std::endl;
        ss << "kf.F\n"
           << kf.F << std::endl;
        ss << "kf.P_last\n"
           << kf.P_last << std::endl;
        ss << "kf.P\n"
           << kf.P << std::endl;
        ss << "kf.I.toDenseMatrix()\n"
           << kf.I << std::endl;
        ss << "kf.P_priori\n"
           << kf.P_priori << std::endl;
        ss << "kf.K\n"
           << kf.K << std::endl;
        // ss << kf.P_priori << std::endl;
        // ss << kf.x << std::endl;
        return ss;
    }

}
