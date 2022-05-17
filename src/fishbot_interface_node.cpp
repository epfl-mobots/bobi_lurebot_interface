#include <ros/ros.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/MotorVelocitiesStamped.h>
#include <bobi_msgs/EnableIR.h>
#include <bobi_msgs/ReturnVelocity.h>
#include <bobi_msgs/ReturnDroppedMsgs.h>
#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/ProximitySensors.h>
#include <bobi_msgs/DroppedMessages.h>
#include <bobi_msgs/DutyCycle.h>

#include <simpleble/SimpleBLE.h>
#include <bobi_fishbot_interface/fishbot_ble_details.hpp>

#include <iostream>
#include <sstream>

#include <chrono>
#include <mutex>

#include <random>

struct FishbotConfig {
    int rate = 65;
    bool enable_ir = false;
    bool ret_vel = false;
    double max_acceleration = 1.75;
    int pwm_duty_cycle_perc = 33;
    int bt_adapter = 0;
    std::string device_uuid;
};

FishbotConfig get_fishbot_config(const std::shared_ptr<ros::NodeHandle> nh)
{
    FishbotConfig cfg;
    nh->param<int>("rate", cfg.rate, cfg.rate);
    nh->param<bool>("enable_ir", cfg.enable_ir, cfg.enable_ir);
    nh->param<bool>("return_velocity", cfg.ret_vel, cfg.ret_vel);
    nh->param<double>("max_acceleration", cfg.max_acceleration, cfg.max_acceleration);
    nh->param<int>("pwm_duty_cycle_perc", cfg.pwm_duty_cycle_perc, cfg.pwm_duty_cycle_perc);
    nh->param<int>("bt_adapter", cfg.bt_adapter, cfg.bt_adapter);
    nh->getParam("device_uuid", cfg.device_uuid);
    return cfg;
}

class BLEInterface {
public:
    BLEInterface(std::shared_ptr<ros::NodeHandle> nh, FishbotConfig cfg)
        : _nh(nh),
          _cfg(cfg),
          _left_motor_cm_per_s(0.),
          _right_motor_cm_per_s(0.),
          _id_counter(0)
    {
        // subs/pubs
        _motor_vel_sub = _nh->subscribe("set_velocities", 1, &BLEInterface::_motor_velocity_cb, this);
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 1);
        _reported_velocities_pub = nh->advertise<bobi_msgs::MotorVelocities>("reported_velocities", 1);
        _dropped_msgs_pub = nh->advertise<bobi_msgs::DroppedMessages>("dropped_msgs", 1);
        _current_motor_vel_pub = nh->advertise<bobi_msgs::MotorVelocitiesStamped>("current_velocities", 1);

        // services
        _enable_ir_srv = _nh->advertiseService("enable_ir", &BLEInterface::_enable_ir_srv_cb, this);
        _enable_ret_vel_srv = _nh->advertiseService("ret_vel", &BLEInterface::_enable_ret_vel_srv_cb, this);
        _enable_ret_dropped_msgs_srv = _nh->advertiseService("ret_dropped_msgs", &BLEInterface::_enable_ret_dropped_msgs_srv_cb, this);
        _set_max_accel_srv = _nh->advertiseService("set_max_acceleration", &BLEInterface::_set_max_accel_srv_cb, this);
        _set_duty_cycle_srv = _nh->advertiseService("set_duty_cycle", &BLEInterface::_set_duty_cycle_srv_cb, this);

        // initialize BLE peripheral
        auto adapter_list = SimpleBLE::Adapter::get_adapters();
        assert(adapter_list.size() && "No adapter was found.");
        _adapter = std::move(adapter_list[cfg.bt_adapter]);

        _adapter.set_callback_on_scan_start([]() { ROS_INFO("BLE scan started."); });
        _adapter.set_callback_on_scan_stop([]() { ROS_INFO("BLE scan stopped."); });
        _adapter.set_callback_on_scan_found([&](SimpleBLE::Peripheral peripheral) {
            // std::cout << peripheral.identifier() << " " << peripheral.address() << std::endl;
            if (/*peripheral.identifier() == DEVICE_NAME &&*/
                peripheral.address() == cfg.device_uuid) {
                _peripheral = peripheral;
                _peripheral.set_callback_on_connected([]() { ROS_INFO("BLE device successfully connected."); });
                _peripheral.set_callback_on_disconnected([]() { ROS_WARN("BLE device disconnected."); });
            }
        });
        _adapter.scan_for(2500);

        assert(_peripheral.initialized());
        _peripheral.connect();
        _init_notifications();
    }

    ~BLEInterface()
    {
        if (_peripheral.is_connected()) {
            _peripheral.disconnect();
        }
    }

    void refresh()
    {
        MotorCmd cmd;
        cmd.cmds[0] = _id_counter;
        cmd.cmds[1] = toUnsigned<uint16_t>(_left_motor_cm_per_s * 10);
        cmd.cmds[2] = toUnsigned<uint16_t>(_right_motor_cm_per_s * 10);
        _send_velocities(cmd);

        if (_id_counter == 100) {
            _id_counter = 0;
        }
        else {
            ++_id_counter;
        }
    }

protected:
    void _init_notifications()
    {
        ROS_INFO("Connected to Fishbot:");
        {
            FishbotName msg;
            SimpleBLE::ByteArray bytes = _peripheral.readBytes(INFO_SRV_UUID, DEVICE_NAME_CHAR_UUID);
            std::copy(bytes.begin(), bytes.end(), msg.bytes);
            ROS_INFO("\tName: %s", msg.cbytes);
        }
        {
            FishbotFWVersion msg;
            SimpleBLE::ByteArray bytes = _peripheral.readBytes(INFO_SRV_UUID, FW_VERSION_CHAR_UUID);
            std::copy(bytes.begin(), bytes.end(), msg.bytes);
            ROS_INFO("\tFW version: %s", msg.cbytes);
        }

        _peripheral.notify(DROPPED_MSGS_SRV_UUID, DROPPED_MSGS_COUNT_CHAR_UUID, [&](SimpleBLE::ByteArray bytes) {
            DroppedMsg msg;
            std::copy(bytes.begin(), bytes.end(), msg.bytes);

            bobi_msgs::DroppedMessages rosmsg;
            rosmsg.dropped = msg.counters[0];
            rosmsg.total = msg.counters[1];
            _dropped_msgs_pub.publish(rosmsg);
        });

        _peripheral.notify(MOTOR_VEL_SRV_UUID, CURRENT_VEL_CHAR_UUID, [&](SimpleBLE::ByteArray bytes) {
            MotorSpeeds msg;
            std::copy(bytes.begin(), bytes.end(), msg.bytes);

            bobi_msgs::MotorVelocitiesStamped rosmsg;
            rosmsg.header.stamp = ros::Time::now();
            rosmsg.vel.left = toSigned<float>(msg.cmds[0]) / 10.;
            rosmsg.vel.right = toSigned<float>(msg.cmds[1]) / 10.;
            _current_motor_vel_pub.publish(rosmsg);
        });
    }

    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        _left_motor_cm_per_s = motor_velocities->left * 100.;
        _right_motor_cm_per_s = motor_velocities->right * 100.;
    }

    bool _enable_ir_srv_cb(
        bobi_msgs::EnableIR::Request& req,
        bobi_msgs::EnableIR::Response& res)
    {
        // TODO:
        ROS_WARN("Not available in this version");
        return true;
    }

    bool _enable_ret_vel_srv_cb(
        bobi_msgs::ReturnVelocity::Request& req,
        bobi_msgs::ReturnVelocity::Response& res)
    {
        _enable_ret_vel(req.enable);
        return true;
    }

    bool _enable_ret_dropped_msgs_srv_cb(
        bobi_msgs::ReturnDroppedMsgs::Request& req,
        bobi_msgs::ReturnDroppedMsgs::Response& res)
    {
        _enable_dropped_msgs(req.enable);
        return true;
    }

    bool _set_max_accel_srv_cb(
        bobi_msgs::MaxAcceleration::Request& req,
        bobi_msgs::MaxAcceleration::Response& res)
    {
        _set_max_accel(req.max_acceleration * 100);
        return true;
    }

    bool _set_duty_cycle_srv_cb(
        bobi_msgs::DutyCycle::Request& req,
        bobi_msgs::DutyCycle::Response& res)
    {
        _set_duty_cycle(req.low_perc);
        return true;
    }

private:
    void _send_velocities(const MotorCmd& cmd)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray bytes(sizeof(cmd.bytes));
        std::copy(cmd.bytes, cmd.bytes + sizeof(cmd.bytes), bytes.begin());
        _peripheral.write_command(MOTOR_VEL_SRV_UUID, DESIRED_VEL_CHAR_UUID, bytes);
    }

    void _enable_ret_vel(uint8_t enable)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {enable};
        _peripheral.write_command(MOTOR_VEL_SRV_UUID, RETURN_CURRENT_VEL_CHAR_UUID, byte);
    }

    void _enable_dropped_msgs(uint8_t enable)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {enable};
        _peripheral.write_command(DROPPED_MSGS_SRV_UUID, RETURN_DROPPED_MSGS_CHAR_UUID, byte);
    }

    void _set_max_accel(uint8_t accel)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {accel};
        _peripheral.write_command(ACCEL_SRV_UUID, MAX_ACCEL_CHAR_UUID, byte);
    }

    void _set_duty_cycle(uint8_t duty)
    {
        const std::lock_guard<std::mutex> lock(_peripheral_lock);
        SimpleBLE::ByteArray byte = {duty};
        _peripheral.write_command(DUTY_CYCLE_SRV_UUID, SET_DUTY_CYCLE_CHAR_UUID, byte);
    }

protected:
    SimpleBLE::Peripheral _peripheral;
    SimpleBLE::Adapter _adapter;

    std::shared_ptr<ros::NodeHandle> _nh;
    FishbotConfig _cfg;

    float _left_motor_cm_per_s;
    float _right_motor_cm_per_s;
    uint16_t _id_counter;
    std::mutex _peripheral_lock;

    ros::Subscriber _motor_vel_sub;
    ros::Publisher _current_motor_vel_pub;
    ros::Publisher _dropped_msgs_pub;
    ros::Publisher _proximity_sensor_pub;
    ros::Publisher _reported_velocities_pub;
    ros::ServiceServer _enable_ir_srv;
    ros::ServiceServer _enable_ret_vel_srv;
    ros::ServiceServer _enable_ret_dropped_msgs_srv;
    ros::ServiceServer _set_max_accel_srv;
    ros::ServiceServer _set_duty_cycle_srv;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fishbot_interface_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    FishbotConfig cfg = get_fishbot_config(nh);
    BLEInterface blei(nh, cfg);

    ros::Rate loop_rate(cfg.rate);
    while (ros::ok()) {
        auto loop_start = std::chrono::high_resolution_clock::now();

        blei.refresh();
        ros::spinOnce();
        loop_rate.sleep();

        auto loop_stop = std::chrono::high_resolution_clock::now();
        std::chrono::microseconds loop_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_stop - loop_start);
        ROS_INFO("Loop time: %ld", loop_time.count());
    }

    return 0;
}