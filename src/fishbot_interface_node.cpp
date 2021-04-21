#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/EnableIR.h>
#include <bobi_msgs/ProximitySensors.h>

#include <iostream>

struct FishbotConfig {
    std::vector<double> led_colour = {0, 0, 255};
    std::string ip = "192.168.0.157";
    int port = 5623;
    int rate = 30;
    bool enable_ir = false;
};

FishbotConfig get_fishbot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    FishbotConfig cfg;
    nh->param<std::vector<double>>("led_colour", cfg.led_colour, cfg.led_colour);
    nh->param<std::string>("ip", cfg.ip, cfg.ip);
    nh->param<int>("port", cfg.port, cfg.port);
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    return cfg;
}

class UDPCom {
public:
    UDPCom(std::shared_ptr<ros::NodeHandle> nh, FishbotConfig cfg)
        : _nh(nh),
          _cfg(cfg),
          _socket(_io_service),
          _endpoint(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(_cfg.ip), _cfg.port))
    {
        _enable_ir = cfg.enable_ir;
        _socket.open(boost::asio::ip::udp::v4());
        _motor_vel_sub = _nh->subscribe("motor_velocities", 5, &UDPCom::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 5);
        _enable_ir_srv = _nh->advertiseService("enable_ir", &UDPCom::_enable_ir_srv_cb, this);
    }

    void send(std::string command)
    {
        boost::system::error_code err;
        auto sent = _socket.send_to(boost::asio::buffer(command), _endpoint, 0, err);
    }

    ~UDPCom()
    {
        _socket.close();
    }

protected:
    void _receive()
    {
        boost::array<char, 64> resp;
        size_t len = _socket.receive_from(boost::asio::buffer(resp), _endpoint);
        std::string proximity_msg(resp.begin(), resp.begin() + len);

        std::vector<std::string> values_str;
        boost::split(values_str, proximity_msg, [](char c) { return c == ':'; });

        bobi_msgs::ProximitySensors ps;
        ps.header.stamp = ros::Time::now();
        std::transform(values_str.begin(), values_str.end(), std::back_inserter(ps.values),
            [](const std::string& str) { return std::stoi(str); });

        _proximity_sensor_pub.publish(ps);
    }

    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        int left_motor_mm_per_s = motor_velocities->left * 1000;
        int right_motor_mm_per_s = motor_velocities->right * 1000;
        std::string cmd = std::to_string(left_motor_mm_per_s) + ":" + std::to_string(right_motor_mm_per_s) + ":" + std::to_string(_enable_ir);

        send(cmd);
        if (_enable_ir) {
            _receive();
        }
    }

    bool _enable_ir_srv_cb(
        bobi_msgs::EnableIR::Request& req,
        bobi_msgs::EnableIR::Response& res)
    {
        _enable_ir = req.enable;
        return true;
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _endpoint;

    std::atomic<bool> _enable_ir;
    ros::Subscriber _motor_vel_sub;
    ros::Publisher _proximity_sensor_pub;
    ros::ServiceServer _enable_ir_srv;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bobi_fishbot_interface");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    FishbotConfig cfg = get_fishbot_config(nh);
    UDPCom udp_talker(nh, cfg);

    ros::Rate loop_rate(cfg.rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}