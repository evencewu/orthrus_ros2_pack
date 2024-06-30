#include <Eigen/Dense>
#include <Eigen/Geometry>

/// @brief ps.这里的variable储存的都是经过转换的标准化数据，并非原始数据
namespace orthrus_control
{
    struct LegImuVariable
    {
        Eigen::Quaterniond attitude;
    };

    struct BodyImuVariable
    {
        Eigen::Quaterniond attitude;
        Eigen::Vector3d acceleration;
        Eigen::Vector3d angle_speed;
    };

    struct MotorVariable
    {
        double position;
        double velocity;
        double acceleration;
    };

    struct LegVariable
    {
        LegImuVariable leg_imu;
    };

    struct OrthrusControlVariable
    {
        LegVariable leg[4];
        BodyImuVariable body_imu;
    };
}