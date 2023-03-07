#include <wind_velocity_observer/ros_handler.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "wind_velocity_observer");

    ros::NodeHandle nh;

    RosHandler rh(nh);

    ros::spin();

    return 0;
}
