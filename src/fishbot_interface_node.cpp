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
#include <mutex>

namespace rpj = rapidjson;

class SimpleSerial {
public:
    SimpleSerial(const std::string& port, unsigned int baud_rate)
        : _io(), _serial(_io, port)
    {
        _serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    void write(std::string s, const std::string delim = "\r")
    {
        s += delim;
        boost::asio::write(_serial, boost::asio::buffer(s.c_str(), s.size()));
    }

    std::string receive()
    {
        return read_until();
    }

    std::string read_until(const std::string delim = "\r")
    {
        boost::asio::streambuf b;
        boost::asio::read_until(_serial, b, delim);
        std::istream is(&b);
        std::string line;
        std::getline(is, line);
        return line;
    }

    ~SimpleSerial()
    {
        _serial.close();
    }

private:
    boost::asio::io_service _io;
    boost::asio::serial_port _serial;
};

struct FishbotConfig {
    std::vector<double> led_colour = {0, 0, 255};
    std::string bt_serial = "/dev/ttyS0";
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
    nh->param<std::string>("bt_serial", cfg.bt_serial, cfg.bt_serial);
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    nh->param<bool>("enable_temp", cfg.enable_temp, cfg.enable_temp);
    nh->param<bool>("return_velocity", cfg.ret_vel, cfg.ret_vel);
    nh->param<double>("max_acceleration", cfg.max_acceleration, cfg.max_acceleration);
    nh->param<double>("max_temperature", cfg.max_temperature, cfg.max_temperature);
    nh->param<bool>("high_current", cfg.high_current, cfg.high_current);
    return cfg;
}

class BTCom {
public:
    BTCom(std::shared_ptr<ros::NodeHandle> nh, FishbotConfig cfg)
        : _nh(nh),
          _cfg(cfg),
          _serial_port{_cfg.bt_serial},
          _ss{_serial_port, 9600}
    {
        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &BTCom::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);
        _temperature_sensor_pub = nh->advertise<bobi_msgs::TemperatureSensors>("temperature_sensors", 1);
        _reported_velocities_pub = nh->advertise<bobi_msgs::MotorVelocities>("reported_velocities", 1);
        _enable_ir_srv = _nh->advertiseService("enable_ir", &BTCom::_enable_ir_srv_cb, this);
        _enable_temp_srv = _nh->advertiseService("enable_temp", &BTCom::_enable_temp_srv_cb, this);
        _enable_ret_vel_srv = _nh->advertiseService("ret_vel", &BTCom::_enable_ret_vel_srv_cb, this);
        _set_max_accel_srv = _nh->advertiseService("set_max_acceleration", &BTCom::_set_max_accel_srv_cb, this);
        _set_max_temp_srv = _nh->advertiseService("set_max_temperature", &BTCom::_set_max_temp_srv_cb, this);
        _set_high_current_srv = _nh->advertiseService("set_high_current", &BTCom::_set_high_current_srv_cb, this);

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
    }

    void communicate()
    {
        // send robot commands
        {
            const std::lock_guard<std::mutex> lock(_json_mtx);
            rpj::StringBuffer cmd_json;
            rpj::Writer<rpj::StringBuffer> writer(cmd_json);
            writer.SetMaxDecimalPlaces(2);
            _json_doc.Accept(writer);
            _ss.write(cmd_json.GetString());
        }

        // receive robot feedback if enabled
        // std::string response = _ss.receive();
        // if (response.length()) {
        //     rpj::Document resp_json_doc;
        //     resp_json_doc.Parse(response.c_str());

        //     if (resp_json_doc.HasMember("ir")) {
        //         const rpj::Value& values = resp_json_doc["ir"];
        //         bobi_msgs::ProximitySensors ps;
        //         ps.header.stamp = ros::Time::now();
        //         for (rpj::SizeType i = 0; i < values.Size(); i++) {
        //             ps.values.push_back(values[i].GetInt());
        //         }
        //         _proximity_sensor_pub.publish(ps);
        //         ROS_INFO("Robot is reporting [IR] = [%d, %d, %d]", ps.values[0], ps.values[1], ps.values[2]);
        //     }

        //     if (resp_json_doc.HasMember("temp")) {
        //         const rpj::Value& values = resp_json_doc["temp"];
        //         bobi_msgs::TemperatureSensors ts;
        //         ts.header.stamp = ros::Time::now();
        //         for (rpj::SizeType i = 0; i < values.Size(); ++i) {
        //             ts.values.push_back(values[i].GetFloat());
        //         }
        //         _temperature_sensor_pub.publish(ts);
        //         ROS_INFO("Robot is reporting [temp] = [%f, %f]", ts.values[0], ts.values[1]);
        //     }

        //     if (resp_json_doc.HasMember("vl") && resp_json_doc.HasMember("vr")) {
        //         const rpj::Value& vl = resp_json_doc["vl"];
        //         const rpj::Value& vr = resp_json_doc["vr"];
        //         bobi_msgs::MotorVelocitiesStamped mv;
        //         mv.header.stamp = ros::Time::now();
        //         mv.vel.left = vl.GetFloat() / 100.;
        //         mv.vel.right = vr.GetFloat() / 100.;
        //         ROS_INFO("Robot is reporting [vl, vr] = [%f, %f]", mv.vel.left, mv.vel.right);
        //     }
        // }
    }

protected:
    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        float left_motor_cm_per_s = motor_velocities->left * 100.;
        float right_motor_cm_per_s = motor_velocities->right * 100.;
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _json_doc["vl"].SetFloat(left_motor_cm_per_s);
        _json_doc["vr"].SetFloat(right_motor_cm_per_s);
    }

    bool _enable_ir_srv_cb(
        bobi_msgs::EnableIR::Request& req,
        bobi_msgs::EnableIR::Response& res)
    {
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _cfg.enable_ir = req.enable;
        _json_doc["ir"].SetBool(_cfg.enable_ir);
        return true;
    }

    bool _enable_temp_srv_cb(
        bobi_msgs::EnableTemp::Request& req,
        bobi_msgs::EnableTemp::Response& res)
    {
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _cfg.enable_temp = req.enable;
        _json_doc["temp"].SetBool(_cfg.enable_temp);
        return true;
    }

    bool _enable_ret_vel_srv_cb(
        bobi_msgs::ReturnVelocity::Request& req,
        bobi_msgs::ReturnVelocity::Response& res)
    {
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _cfg.ret_vel = req.enable;
        _json_doc["ret_vel"].SetBool(_cfg.ret_vel);
        return true;
    }

    bool _set_max_accel_srv_cb(
        bobi_msgs::MaxAcceleration::Request& req,
        bobi_msgs::MaxAcceleration::Response& res)
    {
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _cfg.max_acceleration = req.max_acceleration;
        _json_doc["a"].SetFloat(_cfg.max_acceleration);
        return true;
    }

    bool _set_max_temp_srv_cb(
        bobi_msgs::MaxTemperature::Request& req,
        bobi_msgs::MaxTemperature::Response& res)
    {
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _cfg.max_temperature = req.threshold;
        _json_doc["max_temp"].SetFloat(_cfg.max_temperature);
        return true;
    }

    bool _set_high_current_srv_cb(
        bobi_msgs::HighCurrent::Request& req,
        bobi_msgs::HighCurrent::Response& res)
    {
        const std::lock_guard<std::mutex> lock(_json_mtx);
        _cfg.high_current = req.enable;
        _json_doc["high_current"].SetBool(_cfg.high_current);
        return true;
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;
    std::string _serial_port;
    SimpleSerial _ss;
    boost::array<char, 256> _response_buffer;

    std::mutex _json_mtx;

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

    rpj::Document _json_doc;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fishbot_interface_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    FishbotConfig cfg = get_fishbot_config(nh);
    BTCom btcom(nh, cfg);

    ros::Rate loop_rate(cfg.rate);
    while (ros::ok()) {
        btcom.communicate();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}