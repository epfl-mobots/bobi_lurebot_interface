#include <ros/ros.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/MotorVelocitiesStamped.h>
#include <bobi_msgs/EnableIR.h>
#include <bobi_msgs/ReturnVelocity.h>
#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/ProximitySensors.h>

#include <simpleble/SimpleBLE.h>

#include <iostream>
#include <sstream>

#include <chrono>
#include <mutex>

struct FishbotConfig {
    int rate = 65;
    bool enable_ir = false;
    bool ret_vel = false;
    double max_acceleration = 1.75;
    int pwm_duty_cycle_perc = 33;
};

FishbotConfig get_fishbot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    FishbotConfig cfg;
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    nh->param<bool>("return_velocity", cfg.ret_vel, cfg.ret_vel);
    nh->param<double>("max_acceleration", cfg.max_acceleration, cfg.max_acceleration);
    nh->param<int>("pwm_duty_cycle_perc", cfg.pwm_duty_cycle_perc, cfg.pwm_duty_cycle_perc);
    return cfg;
}

class BLEInterface {
public:
    BLEInterface(std::shared_ptr<ros::NodeHandle> nh, FishbotConfig cfg)
        : _nh(nh),
          _cfg(cfg)
    {
        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &BLEInterface::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);
        _reported_velocities_pub = nh->advertise<bobi_msgs::MotorVelocities>("reported_velocities", 1);
        _enable_ir_srv = _nh->advertiseService("enable_ir", &BLEInterface::_enable_ir_srv_cb, this);
        _enable_ret_vel_srv = _nh->advertiseService("ret_vel", &BLEInterface::_enable_ret_vel_srv_cb, this);
        _set_max_accel_srv = _nh->advertiseService("set_max_acceleration", &BLEInterface::_set_max_accel_srv_cb, this);
    }

    void refresh()
    {
        // TODO: ble
    }

protected:
    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        float left_motor_cm_per_s = motor_velocities->left * 100.;
        float right_motor_cm_per_s = motor_velocities->right * 100.;
        // TODO:
    }

    bool _enable_ir_srv_cb(
        bobi_msgs::EnableIR::Request& req,
        bobi_msgs::EnableIR::Response& res)
    {
        // TODO:
        return true;
    }

    bool _enable_ret_vel_srv_cb(
        bobi_msgs::ReturnVelocity::Request& req,
        bobi_msgs::ReturnVelocity::Response& res)
    {
        // TODO:
        return true;
    }

    bool _set_max_accel_srv_cb(
        bobi_msgs::MaxAcceleration::Request& req,
        bobi_msgs::MaxAcceleration::Response& res)
    {
        // TODO:
        return true;
    }

    SimpleBLE::Peripheral _peripheral;

    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;

    ros::Subscriber _motor_vel_sub;
    ros::Publisher _proximity_sensor_pub;
    ros::Publisher _reported_velocities_pub;
    ros::ServiceServer _enable_ir_srv;
    ros::ServiceServer _enable_ret_vel_srv;
    ros::ServiceServer _set_max_accel_srv;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fishbot_interface_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    FishbotConfig cfg = get_fishbot_config(nh);
    BLEInterface blei(nh, cfg);

    ros::Rate loop_rate(cfg.rate);
    while (ros::ok()) {
        blei.refresh();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}