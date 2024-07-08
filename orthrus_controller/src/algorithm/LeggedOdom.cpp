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

        Eigen::Vector3d acc_data = robot->body_imu.orientation * robot->body_imu.linear_acceleration + odom->gravity;
        acc_data = AverageFilter(acc_data, imu_velocity_filter_list_); // 过一个均值滤波
        odom->imu.linear_acceleration = acc_data;

        // for (int i = 0; i < 3; i++)
        //{
        //     if (acc_data[i] <= 0.15 && acc_data[i] >= -0.15)
        //     {
        //         acc_data[i] = 0;
        //     }
        // }

        odom->imu.orientation = robot->body_imu.orientation;
        odom->imu.angular_velocity = robot->body_imu.angular_velocity;

        //;
        // 原始数据滤波载入
        odom->euler = Quaternion2Euler(odom->imu.orientation);

        odom->dt = (double)(duration.nanoseconds()) / 1000000000;

        double dt = odom->dt;

        OdomFilterUpdate();

        orthrus_interfaces_->odom_state.touch_state[0].sensor = true;
        orthrus_interfaces_->odom_state.touch_state[1].sensor = true;
        orthrus_interfaces_->odom_state.touch_state[2].sensor = true;
        orthrus_interfaces_->odom_state.touch_state[3].sensor = true;

        LeggedFilterUpdate();

        orthrus_interfaces_->odom_state.position = orthrus_interfaces_->odom_state.foot_position;
        // orthrus_interfaces_->velocity
        // orthrus_interfaces_->angular_velocity
    }

    Eigen::Vector3d LeggedOdom::Quaternion2Euler(Eigen::Quaterniond quat)
    {
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        double roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
        double pitch = std::asin(-rotation_matrix(2, 0));
        double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

        Eigen::Vector3d euler_angles;
        euler_angles << roll, pitch, yaw;
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

            kf.P.setIdentity(); // 将 P 初始化为单位矩阵
            kf.P *= 0.1;

            kf.H << 0, 0, 0, 0, 1, 0,
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

        //--
        double a_x = odom->imu.linear_acceleration[0];
        double a_y = odom->imu.linear_acceleration[1];

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

            kf.K = (kf.P_priori * kf.H.transpose()) * (kf.H * kf.P_priori * kf.H.transpose() + kf.R).inverse();

            kf.x = kf.x_priori + kf.K * (kf.z - kf.H * kf.x_priori);

            kf.P = (kf.I - kf.K * kf.H) * kf.P_last;

            orthrus_interfaces_->odom_state.imu_position[0] = kf.x[0];
            orthrus_interfaces_->odom_state.imu_position[1] = kf.x[1];
            orthrus_interfaces_->odom_state.imu_position[2] = 0;

            orthrus_interfaces_->odom_state.imu_velocity[0] = kf.x[2];
            orthrus_interfaces_->odom_state.imu_velocity[1] = kf.x[3];
            orthrus_interfaces_->odom_state.imu_velocity[2] = 0;

            orthrus_interfaces_->odom_state.imu_acceleration[0] = kf.x[4];
            orthrus_interfaces_->odom_state.imu_acceleration[1] = kf.x[5];
            orthrus_interfaces_->odom_state.imu_acceleration[2] = 0;
        }
    }

    Eigen::Vector3d LeggedOdom::AverageFilter(const Eigen::Vector3d new_data, std::vector<Eigen::Vector3d> &data_list)
    {
        for (int i = (data_list.size() - 1); i > 0; i--)
        {
            data_list[i] = data_list[i - 1];
        }

        data_list[0] = new_data;

        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (int i = 0; i < data_list.size(); i++)
        {
            sum += data_list[i];
        }

        return sum / data_list.size();
    }

    Eigen::Vector3d LeggedOdom::MediumFilter(const Eigen::Vector3d new_data, std::vector<Eigen::Vector3d> &data_list)
    {
        double sort_list[3][data_list.size()];

        for (int i = (data_list.size() - 1); i > 0; i--)
        {
            data_list[i] = data_list[i - 1];
        }

        data_list[0] = new_data;

        int num = 0;
        for (const auto &vec : data_list)
        {
            sort_list[0][num] = vec[0];
            sort_list[1][num] = vec[1];
            sort_list[2][num] = vec[2];
            num++;
        }

        std::sort(&sort_list[0][0], &sort_list[0][data_list.size() - 1]);
        std::sort(&sort_list[1][0], &sort_list[1][data_list.size() - 1]);
        std::sort(&sort_list[2][0], &sort_list[2][data_list.size() - 1]);

        Eigen::Vector3d medium;
        medium << sort_list[0][data_list.size() / 2], sort_list[1][data_list.size() / 2], sort_list[2][data_list.size() / 2];
        return medium;
    }

    std::stringstream LeggedOdom::OdomFilterLog()
    {
        std::stringstream ss;

        ss << "kf.x_priori\n"
           << kf.x_priori << std::endl;
        ss << "kf.x\n"
           << kf.x << std::endl;
        ss << "kf.F\n"
           << kf.F << std::endl;
        ss << "kf.P_last\n"
           << kf.P_last << std::endl;
        ss << "kf.P_last.inverse()\n"
           << kf.P_last.inverse() << std::endl;
        ss << "kf.P\n"
           << kf.P << std::endl;
        ss << "kf.P_priori\n"
           << kf.P_priori << std::endl;
        ss << "kf.K\n"
           << kf.K << std::endl;
        // ss << kf.P_priori << std::endl;
        // ss << kf.x << std::endl;
        return ss;
    }

    void LeggedOdom::LeggedFilterUpdate()
    {
        Eigen::Vector3d center_all = Eigen::Vector3d::Zero();
        int leg_num = 0;
        for (int i = 0; i < 4; i++)
        {
            if (orthrus_interfaces_->odom_state.touch_state[i].sensor)
            {
                leg_num++;
                center_all += orthrus_interfaces_->odom_state.touch_state[i].touch_position;
            }
        }

        Eigen::Vector3d center = center_all / leg_num;

        orthrus_interfaces_->odom_state.foot_position[0] = -center[0];
        orthrus_interfaces_->odom_state.foot_position[1] = -center[1];
        orthrus_interfaces_->odom_state.foot_position[2] = -center[2];
    }

}
