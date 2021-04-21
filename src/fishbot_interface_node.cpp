#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

#include <iostream>

struct FishbotConfig {
    std::vector<double> led_colour = {0, 0, 255};
    std::string ip = "192.168.1.157";
    int port = 5623;
    int rate = 30;
};

FishbotConfig get_fishbot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    FishbotConfig cfg;
    nh->param<std::vector<double>>("led_colour", cfg.led_colour, cfg.led_colour);
    nh->param<std::string>("ip", cfg.ip, cfg.ip);
    nh->param<int>("port", cfg.port, cfg.port);
    nh->param<int>("rate", cfg.rate, cfg.rate);
    return cfg;
}

class UDPTalker {
public:
    UDPTalker(std::shared_ptr<ros::NodeHandle> nh, FishbotConfig cfg)
        : _nh(nh),
          _cfg(cfg),
          _socket(_io_service),
          _endpoint(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(_cfg.ip), _cfg.port))
    {
        _socket.open(boost::asio::ip::udp::v4());
    }

    void send(std::string command)
    {
        boost::system::error_code err;
        auto sent = _socket.send_to(boost::asio::buffer(command), _endpoint, 0, err);
    }

    ~UDPTalker()
    {
        _socket.close();
    }

protected:
    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _endpoint;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bobi_fishbot_interface");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    FishbotConfig cfg = get_fishbot_config(nh);
    UDPTalker udp_talker(nh, cfg);

    ros::Rate loop_rate(cfg.rate);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}