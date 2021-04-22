#include <ros/ros.h>

#include <bobi_msgs/MotorVelocities.h>
#include <bobi_msgs/ProximitySensors.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <sstream>

class DummyFishbotInterface {
public:
    DummyFishbotInterface(std::shared_ptr<ros::NodeHandle> nh)
        : _nh(nh)
    {
        _motor_vel_sub = _nh->subscribe("set_velocities", 5, &DummyFishbotInterface::_motor_velocity_cb, this);

        _cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // TODO: not implemented yet
        _proximity_sensor_pub = nh->advertise<bobi_msgs::ProximitySensors>("proximity_sensors", 5);
    }

protected:
    void _motor_velocity_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
    {
        double Vr = motor_velocities->right;
        double Vl = motor_velocities->left;
        double l = 0.0451; // This is the distance between the wheels for the fisbot_v5
        double w = (Vr - Vl) / l; // angular velocity
        double v = (Vr + Vl) / 2;
        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        _cmd_vel_pub.publish(msg);
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    ros::Subscriber _motor_vel_sub;
    ros::Publisher _cmd_vel_pub;
    ros::Publisher _proximity_sensor_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bobi_fishbot_dummy_interface");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    DummyFishbotInterface fishbot(nh);

    int rate;
    nh->param<int>("rate", rate, 30);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}