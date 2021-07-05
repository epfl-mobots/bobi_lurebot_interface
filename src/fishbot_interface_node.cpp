#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/EnableIR.h>
#include <bobi_msgs/EnableTemp.h>
#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/ProximitySensors.h>
#include <bobi_msgs/TemperatureSensors.h>

#include <iostream>
#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

namespace rpj = rapidjson;
struct FishbotConfig {
    std::vector<double> led_colour = {0, 0, 255};
    std::string ip = "192.168.0.157";
    int port = 5623;
    int rate = 30;
    bool enable_ir = false;
    bool enable_temp = false;
    double max_acceleration = 1.75;
};

FishbotConfig get_fishbot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    FishbotConfig cfg;
    nh->param<std::vector<double>>("led_colour", cfg.led_colour, cfg.led_colour);
    nh->param<std::string>("ip", cfg.ip, cfg.ip);
    nh->param<int>("port", cfg.port, cfg.port);
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    nh->param<bool>("enable_ir", cfg.enable_temp, cfg.enable_temp);
    nh->param<double>("max_acceleration", cfg.max_acceleration, cfg.max_acceleration);
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
        _enable_temp = cfg.enable_temp;
        _socket.open(boost::asio::ip::udp::v4());
        _motor_vel_sub = _nh->subscribe("set_velocities", 3, &UDPCom::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);
        _temperature_sensor_pub = nh->advertise<bobi_msgs::TemperatureSensors>("temperature_sensors", 1);
        _enable_ir_srv = _nh->advertiseService("enable_ir", &UDPCom::_enable_ir_srv_cb, this);
        _enable_temp_srv = _nh->advertiseService("enable_temp", &UDPCom::_enable_temp_srv_cb, this);
        _set_max_accel_srv = _nh->advertiseService("set_max_acceleration", &UDPCom::_set_max_accel_srv_cb, this);

        // initialize json doc
        _json_doc.SetObject();
        rpj::Document::AllocatorType& allocator = _json_doc.GetAllocator();
        rpj::Value object1(rpj::kObjectType);

        _json_doc.AddMember("vl", 0., allocator);
        _json_doc.AddMember("vr", 0., allocator);
        if (_enable_ir) {
            _json_doc.AddMember("ir", _cfg.enable_ir, allocator);
        }
        if (_enable_temp) {
            _json_doc.AddMember("temp", _cfg.enable_temp, allocator);
        }
        _json_doc.AddMember("a", _cfg.max_acceleration, allocator);
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
        std::string response(resp.begin(), resp.begin() + len);

        rpj::Document resp_json_doc;
        resp_json_doc.Parse(response.c_str());

        if (resp_json_doc.HasMember("ir")) {
            const rpj::Value& values = resp_json_doc["ir"];
            bobi_msgs::ProximitySensors ps;
            ps.header.stamp = ros::Time::now();
            for (rpj::SizeType i = 0; i < values.Size(); i++) {
                ps.values.push_back(values[i].GetInt());
            }
            _proximity_sensor_pub.publish(ps);
        }

        if (resp_json_doc.HasMember("temp")) {
            const rpj::Value& values = resp_json_doc["temp"];
            bobi_msgs::TemperatureSensors ts;
            ts.header.stamp = ros::Time::now();
            for (rpj::SizeType i = 0; i < values.Size(); i++) {
                ts.values.push_back(values[i].GetFloat());
            }
            _temperature_sensor_pub.publish(ts);
        }
    }

    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        float left_motor_cm_per_s = motor_velocities->left * 100.;
        float right_motor_cm_per_s = motor_velocities->right * 100.;

        _json_doc["vl"].SetFloat(left_motor_cm_per_s);
        _json_doc["vr"].SetFloat(right_motor_cm_per_s);

        if (_enable_ir) {
            if (!_json_doc.HasMember("ir")) {
                _json_doc.AddMember("ir", _cfg.enable_ir, _json_doc.GetAllocator());
            }
            _json_doc["ir"].SetBool(_enable_ir);
        }
        else if (_json_doc.HasMember("ir")) {
            _json_doc["ir"].SetBool(_enable_ir);
        }

        if (_enable_temp) {
            if (!_json_doc.HasMember("temp")) {
                _json_doc.AddMember("temp", _cfg.enable_temp, _json_doc.GetAllocator());
            }
            _json_doc["temp"].SetBool(_enable_temp);
        }
        else if (_json_doc.HasMember("temp")) {
            _json_doc["temp"].SetBool(_enable_temp);
        }

        _json_doc["a"].SetFloat(_cfg.max_acceleration);

        rpj::StringBuffer cmd_json;
        rpj::Writer<rpj::StringBuffer> writer(cmd_json);
        _json_doc.Accept(writer);

        if (!_enable_ir && _json_doc.HasMember("ir")) {
            _json_doc.RemoveMember("ir");
        }

        if (!_enable_temp && _json_doc.HasMember("temp")) {
            _json_doc.RemoveMember("temp");
        }

        send(cmd_json.GetString());
        if (_enable_ir || _enable_temp) {
            _receive();
        }
    }

    bool _enable_ir_srv_cb(
        bobi_msgs::EnableIR::Request& req,
        bobi_msgs::EnableIR::Response& res)
    {
        _enable_ir = req.enable;
        _cfg.enable_ir = _enable_ir;
        return true;
    }

    bool _enable_temp_srv_cb(
        bobi_msgs::EnableTemp::Request& req,
        bobi_msgs::EnableTemp::Response& res)
    {
        _enable_temp = req.enable;
        _cfg.enable_temp = _enable_temp;
        return true;
    }

    bool _set_max_accel_srv_cb(
        bobi_msgs::MaxAcceleration::Request& req,
        bobi_msgs::MaxAcceleration::Response& res)
    {
        _cfg.max_acceleration = req.max_acceleration;
        return true;
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _endpoint;

    std::atomic<bool> _enable_ir;
    std::atomic<bool> _enable_temp;
    ros::Subscriber _motor_vel_sub;
    ros::Publisher _proximity_sensor_pub;
    ros::Publisher _temperature_sensor_pub;
    ros::ServiceServer _enable_ir_srv;
    ros::ServiceServer _enable_temp_srv;
    ros::ServiceServer _set_max_accel_srv;

    rpj::Document _json_doc;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fishbot_interface_node");
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