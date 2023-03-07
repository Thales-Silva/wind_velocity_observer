#include <wind_velocity_observer/ros_handler.hpp>

const double RosHandler::RPM_TO_RADS = 2.0 * M_PI / 60.0;

RosHandler::RosHandler(ros::NodeHandle &nh) :
    nh(nh)
{
    std::string temp_str;
    std::string node_name = ros::this_node::getName();

    if (!nh.getParam(node_name + "/drone_pose_topic", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter drone_pose_topic doesn't exist."
                                          "Please, set the parameter in congig.yaml");
    drone_pose_sub.subscribe(nh, temp_str, 5);
    temp_str.clear();

    if (!nh.getParam(node_name + "/drone_twist_topic", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter drone_twist_topic doesn't exist."
                                          "Please, set the parameter in congig.yaml");
    drone_twist_sub.subscribe(nh, temp_str, 5);
    temp_str.clear();

    if (!nh.getParam(node_name + "/esc_telemetry_topic", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter rotor_i_topic."
                                          "Please, set the parameter in congig.yaml");
    esc_telemetry_sub.subscribe(nh, temp_str, 5);
    temp_str.clear();

    double temp_d;
    if (nh.getParam(node_name + "/total_mass", temp_d) && temp_d > 0)
    {
        mass = temp_d;
        mass_inverse = 1/temp_d;
    }
    else
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter total_mass."
                                          "Please, set the parameter in congig.yaml");

    if (nh.getParam(node_name + "/observer_gain", temp_d) && temp_d > 0)
        c = temp_d;
    else
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter observer_gain."
                                          "Please, set the parameter in congig.yaml");

    if (nh.getParam(node_name + "/inversion_gain", temp_d) && temp_d > 0)
        inversion_gain = temp_d;
    else
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter inversion_gain."
                                          "Please, set the parameter in congig.yaml");

    if (nh.getParam(node_name + "/thrust_coefficient", temp_d) && temp_d > 0)
        thrust_coefficient = temp_d;
    else
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter thrust_coefficient."
                                          "Please, set the parameter in congig.yaml");

    if (nh.getParam(node_name + "/rotor_drag_coefficient", temp_d) && temp_d > 0)
        rotor_drag_coefficient = temp_d;
    else
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter rotor_drag_coefficient."
                                          "Please, set the parameter in congig.yaml");

    std::vector<double> temp_vec;
    if (nh.getParam(node_name + "/effort_coefficients", temp_vec))
    {
        effort_coeffiecients = Eigen::Matrix3d(temp_vec.data());
        effort_coeffiecients_inv = effort_coeffiecients.inverse();
    }
    else
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter effort_coefficients."
                                          "Please, set the parameter in congig.yaml");

    if (!nh.getParam(node_name + "/estimated_relative_velocity_topic", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter estimated_relative_velocity_topic."
                                          "Please, set the parameter in congig.yaml");
    air_relative_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>(temp_str, 5);
    temp_str.clear();

    if (!nh.getParam(node_name + "/estimated_wind_velocity_topic", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] couldn't find parameter estimated_wind_velocity_topic."
                                          "Please, set the parameter in congig.yaml");
    wind_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>(temp_str, 5);
    temp_str.clear();

    _sync_ptr.reset(new Sync(MySyncPolicy(5), drone_pose_sub,
                             drone_twist_sub,
                             esc_telemetry_sub));
    _sync_ptr->registerCallback(boost::bind(&RosHandler::mainCallback, this, _1, _2, _3));

    std::cout << BOLD_YELLOW "[ WARN] all topics configured." << std::endl;
}

void RosHandler::mainCallback(const geometry_msgs::PoseStamped::ConstPtr &pose,
                              const geometry_msgs::TwistStamped::ConstPtr &twist,
                              const mavros_msgs::ESCTelemetry::ConstPtr &motor_speed)
{
    Eigen::Quaterniond orientation({pose->pose.orientation.w,
                                    pose->pose.orientation.x,
                                    pose->pose.orientation.y,
                                    pose->pose.orientation.z});

    Eigen::Vector3d line_vel({twist->twist.linear.x,
                              twist->twist.linear.y,
                              twist->twist.linear.z});

    std::vector<double> motor_speed_vec;
    for (auto i = motor_speed->esc_telemetry.begin(), end = motor_speed->esc_telemetry.end(); i != end; i++)
        motor_speed_vec.push_back( i->rpm * RPM_TO_RADS );

    updateObserver(orientation, line_vel, motor_speed_vec);

    geometry_msgs::Vector3Stamped r_vel_msg;
    r_vel_msg.header.stamp = ros::Time::now();
    r_vel_msg.header.frame_id = "world";
    r_vel_msg.vector.x = air_relative_velocity[0];
    r_vel_msg.vector.y = air_relative_velocity[1];
    r_vel_msg.vector.z = air_relative_velocity[2];
    air_relative_velocity_pub.publish(r_vel_msg);

    geometry_msgs::Vector3Stamped w_vel_msg;
    w_vel_msg.header.stamp = ros::Time::now();
    w_vel_msg.header.frame_id = "world";
    w_vel_msg.vector.x = wind_velocity[0];
    w_vel_msg.vector.y = wind_velocity[1];
    w_vel_msg.vector.z = wind_velocity[2];
    wind_velocity_pub.publish(w_vel_msg);
}

RosHandler::~RosHandler()
{
    std::cerr << BOLD_YELLOW "[ WARN] shutting down node!" << std::endl;
}
