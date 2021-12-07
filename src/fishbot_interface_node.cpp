#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/MotorVelocitiesStamped.h>
#include <bobi_msgs/EnableIR.h>
#include <bobi_msgs/EnableTemp.h>
#include <bobi_msgs/ReturnVelocity.h>
#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/MaxTemperature.h>
#include <bobi_msgs/ProximitySensors.h>
#include <bobi_msgs/TemperatureSensors.h>
#include <bobi_msgs/HighCurrent.h>

#include <iostream>
#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <chrono>

namespace rpj = rapidjson;

struct FishbotConfig {
    std::vector<double> led_colour = {0, 0, 255};
    std::string ip = "192.168.4.157";
    int port = 5623;
    int rate = 30;
    bool enable_ir = false;
    bool enable_temp = false;
    bool ret_vel = false;
    double max_acceleration = 1.75;
    double max_temperature = 100.0;
    bool high_current = false;
};

FishbotConfig get_fishbot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    FishbotConfig cfg;
    nh->param<std::vector<double>>("led_colour", cfg.led_colour, cfg.led_colour);
    nh->param<std::string>("ip", cfg.ip, cfg.ip);
    nh->param<int>("port", cfg.port, cfg.port);
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    nh->param<bool>("enable_temp", cfg.enable_temp, cfg.enable_temp);
    nh->param<bool>("return_velocity", cfg.ret_vel, cfg.ret_vel);
    nh->param<double>("max_acceleration", cfg.max_acceleration, cfg.max_acceleration);
    nh->param<double>("max_temperature", cfg.max_temperature, cfg.max_temperature);
    nh->param<bool>("high_current", cfg.high_current, cfg.high_current);
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
        _ret_vel = cfg.ret_vel;
        _high_current = cfg.high_current;
        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &UDPCom::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);
        _temperature_sensor_pub = nh->advertise<bobi_msgs::TemperatureSensors>("temperature_sensors", 1);
        _reported_velocities_pub = nh->advertise<bobi_msgs::MotorVelocities>("reported_velocities", 1);
        _enable_ir_srv = _nh->advertiseService("enable_ir", &UDPCom::_enable_ir_srv_cb, this);
        _enable_temp_srv = _nh->advertiseService("enable_temp", &UDPCom::_enable_temp_srv_cb, this);
        _enable_ret_vel_srv = _nh->advertiseService("ret_vel", &UDPCom::_enable_ret_vel_srv_cb, this);
        _set_max_accel_srv = _nh->advertiseService("set_max_acceleration", &UDPCom::_set_max_accel_srv_cb, this);
        _set_max_temp_srv = _nh->advertiseService("set_max_temperature", &UDPCom::_set_max_temp_srv_cb, this);
        _set_high_current_srv = _nh->advertiseService("set_high_current", &UDPCom::_set_high_current_srv_cb, this);

        // initialize json doc
        _json_doc.SetObject();
        rpj::Document::AllocatorType& allocator = _json_doc.GetAllocator();
        rpj::Value object1(rpj::kObjectType);

        _json_doc.AddMember("vl", 0., allocator);
        _json_doc.AddMember("vr", 0., allocator);
        _json_doc.AddMember("ir", _cfg.enable_ir, allocator);
        _json_doc.AddMember("temp", _cfg.enable_temp, allocator);
        _json_doc.AddMember("ret_vel", _cfg.ret_vel, allocator);
        _json_doc.AddMember("a", _cfg.max_acceleration, allocator);
        _json_doc.AddMember("max_temp", _cfg.max_temperature, allocator);
        _json_doc.AddMember("high_current", _cfg.high_current, allocator);

        _socket.open(boost::asio::ip::udp::v4());
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
        _socket.non_blocking(true);
        boost::system::error_code err = boost::asio::error::would_block;
        size_t len = _socket.receive_from(boost::asio::buffer(_response_buffer), _endpoint, 0, err);
        _socket.non_blocking(false);

        if (len > 0) {
            std::string response(_response_buffer.begin(), _response_buffer.begin() + len);
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
                ROS_INFO("Robot is reporting [IR] = [%d, %d, %d]", ps.values[0], ps.values[1], ps.values[2]);
            }

            if (resp_json_doc.HasMember("temp")) {
                const rpj::Value& values = resp_json_doc["temp"];
                bobi_msgs::TemperatureSensors ts;
                ts.header.stamp = ros::Time::now();
                for (rpj::SizeType i = 0; i < values.Size(); ++i) {
                    ts.values.push_back(values[i].GetFloat());
                }
                _temperature_sensor_pub.publish(ts);
                ROS_INFO("Robot is reporting [temp] = [%f, %f]", ts.values[0], ts.values[1]);
            }

            if (resp_json_doc.HasMember("vl") && resp_json_doc.HasMember("vr")) {
                const rpj::Value& vl = resp_json_doc["vl"];
                const rpj::Value& vr = resp_json_doc["vr"];
                bobi_msgs::MotorVelocitiesStamped mv;
                mv.header.stamp = ros::Time::now();
                mv.vel.left = vl.GetFloat() / 100.;
                mv.vel.right = vr.GetFloat() / 100.;
                ROS_INFO("Robot is reporting [vl, vr] = [%f, %f]", mv.vel.left, mv.vel.right);
            }
        }
        else {
            ROS_WARN("Did not receive feedback");
        }
    }

    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        _prev_header = ros::Time::now();

        float left_motor_cm_per_s = motor_velocities->left * 100.;
        float right_motor_cm_per_s = motor_velocities->right * 100.;

        _json_doc["vl"].SetFloat(left_motor_cm_per_s);
        _json_doc["vr"].SetFloat(right_motor_cm_per_s);
        _json_doc["ir"].SetBool(_enable_ir);
        _json_doc["temp"].SetBool(_enable_temp);
        _json_doc["ret_vel"].SetBool(_ret_vel);
        _json_doc["a"].SetFloat(_cfg.max_acceleration);
        _json_doc["max_temp"].SetFloat(_cfg.max_temperature);
        _json_doc["high_current"].SetBool(_high_current);

        rpj::StringBuffer cmd_json;
        rpj::Writer<rpj::StringBuffer> writer(cmd_json);
        writer.SetMaxDecimalPlaces(2);
        _json_doc.Accept(writer);

        send(cmd_json.GetString());
        if (_enable_ir || _enable_temp || _ret_vel) {
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

    bool _enable_ret_vel_srv_cb(
        bobi_msgs::ReturnVelocity::Request& req,
        bobi_msgs::ReturnVelocity::Response& res)
    {
        _ret_vel = req.enable;
        _cfg.ret_vel = _ret_vel;
        return true;
    }

    bool _set_max_accel_srv_cb(
        bobi_msgs::MaxAcceleration::Request& req,
        bobi_msgs::MaxAcceleration::Response& res)
    {
        _cfg.max_acceleration = req.max_acceleration;
        return true;
    }

    bool _set_max_temp_srv_cb(
        bobi_msgs::MaxTemperature::Request& req,
        bobi_msgs::MaxTemperature::Response& res)
    {
        _cfg.max_temperature = req.threshold;
        return true;
    }

    bool _set_high_current_srv_cb(
        bobi_msgs::HighCurrent::Request& req,
        bobi_msgs::HighCurrent::Response& res)
    {
        _high_current = true;
        _cfg.high_current = req.enable;
        return true;
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _endpoint;
    boost::array<char, 256> _response_buffer;

    std::atomic<bool> _enable_ir;
    std::atomic<bool> _enable_temp;
    std::atomic<bool> _ret_vel;
    std::atomic<bool> _high_current;
    ros::Subscriber _motor_vel_sub;
    ros::Publisher _proximity_sensor_pub;
    ros::Publisher _temperature_sensor_pub;
    ros::Publisher _reported_velocities_pub;
    ros::ServiceServer _enable_ir_srv;
    ros::ServiceServer _enable_temp_srv;
    ros::ServiceServer _enable_ret_vel_srv;
    ros::ServiceServer _set_max_accel_srv;
    ros::ServiceServer _set_max_temp_srv;
    ros::ServiceServer _set_high_current_srv;

    ros::Time _prev_header;

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