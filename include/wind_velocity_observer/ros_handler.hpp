#ifndef ROS_HANDLER_HPP
#define ROS_HANDLER_HPP

#define BOLD_RED "\033[1;31m"
#define BOLD_GREEN "\033[1;32m"
#define BOLD_YELLOW "\033[1;33m"

#include <ros/ros.h>

#include <wind_velocity_observer/wind_velocity_observer.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/ESCTelemetry.h>

#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/bind.hpp>

class RosHandler : public WindVelocityObserver
{
    private:
        ros::NodeHandle &nh;

        message_filters::Subscriber<geometry_msgs::PoseStamped> drone_pose_sub;
        message_filters::Subscriber<geometry_msgs::TwistStamped> drone_twist_sub;
        message_filters::Subscriber<mavros_msgs::ESCTelemetry> esc_telemetry_sub;

        ros::Publisher air_relative_velocity_pub;
        ros::Publisher wind_velocity_pub;

        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped,
        mavros_msgs::ESCTelemetry> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;

        boost::shared_ptr<Sync> _sync_ptr;

    public:
        RosHandler(ros::NodeHandle &nh);
        ~RosHandler();

        void mainCallback(const geometry_msgs::PoseStamped::ConstPtr &pose,
                          const geometry_msgs::TwistStamped::ConstPtr &twist,
                          const mavros_msgs::ESCTelemetry::ConstPtr &motor_speed);

        static const double RPM_TO_RADS;
};

#endif // ROS_HANDLER_HPP
