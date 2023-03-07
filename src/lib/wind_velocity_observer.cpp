#include <wind_velocity_observer/wind_velocity_observer.hpp>

const double WindVelocityObserver::STANDARD_GRAVITY = 9.80665;

WindVelocityObserver::WindVelocityObserver() :
    psi(Eigen::Vector3d::Zero()),
    rotor_drag_coefficient(0),
    inversion_gain(0),
    wind_velocity(Eigen::Vector3d::Zero()),
    air_relative_velocity(Eigen::Vector3d::Zero())
{

}

void WindVelocityObserver::updateObserver(Eigen::Quaterniond orientation,
                                          Eigen::Vector3d linear_velocity,
                                          std::vector<double> motor_speed)
{
    boost::chrono::milliseconds dt = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - last_sample);
    double dt_secs = dt.count()/1000.0;
    last_sample = boost::chrono::steady_clock::now();

    Eigen::Matrix3d rotation = orientation.toRotationMatrix();

    Eigen::Vector3d thrust_world;
    Eigen::Matrix3d rotor_drag_matrix;

    getThrustAndRotorDrag(motor_speed, rotation, thrust_world, rotor_drag_matrix);

    Eigen::Vector3d p = c * linear_velocity;
    Eigen::Matrix3d L = c * mass_inverse * Eigen::Matrix3d::Identity();
    Eigen::Vector3d estimated_force = psi + p;
    psi += dt_secs * (-L * psi - L * (-mass * STANDARD_GRAVITY * Eigen::Vector3d::UnitZ() + thrust_world + p));

    Eigen::Matrix3d gamma = rotation * effort_coeffiecients * rotation.transpose();

    Eigen::Vector3d evaluated_force = - (gamma * air_relative_velocity.norm() + rotor_drag_matrix) * air_relative_velocity;
    Eigen::Vector3d error = estimated_force - evaluated_force;

    Eigen::Matrix3d jacobian = -(gamma * evaluateVectorDerivative() + rotor_drag_matrix);
    air_relative_velocity += dt_secs * inversion_gain * jacobian.transpose() * error;

    wind_velocity = linear_velocity - air_relative_velocity;
}

void WindVelocityObserver::getThrustAndRotorDrag(std::vector<double> spin_rate,
                                                 Eigen::Matrix3d rotation_matrix,
                                                 Eigen::Vector3d &thrust,
                                                 Eigen::Matrix3d &rotor_drag_matrix)
{
    double spin_rate_sum = 0.0;
    double spin_rate_sum_squared = 0.0;

    for (auto i = spin_rate.begin(), end = spin_rate.end(); i != end; i++)
    {
        spin_rate_sum += std::abs(*i);
        spin_rate_sum_squared += (*i) * (*i);
    }

    thrust = rotation_matrix *
             Eigen::Vector3d::UnitZ() *
             spin_rate_sum_squared *
             thrust_coefficient;

    Eigen::Vector3d rotated_axis = rotation_matrix * Eigen::Vector3d::UnitZ();
    rotor_drag_matrix = (Eigen::Matrix3d::Identity() - rotated_axis * rotated_axis.transpose()) *
                        rotor_drag_coefficient *
                        spin_rate_sum;
}


Eigen::Matrix3d WindVelocityObserver::evaluateVectorDerivative()
{
    double norm = air_relative_velocity.norm();
    if (norm != 0.0)
    {
        double norm_squared = norm * norm;

        double j11 = norm_squared + air_relative_velocity(0) * air_relative_velocity(0);
        double j22 = norm_squared + air_relative_velocity(1) * air_relative_velocity(1);
        double j33 = norm_squared + air_relative_velocity(2) * air_relative_velocity(2);

        double j12 = air_relative_velocity(0) * air_relative_velocity(1);
        double j13 = air_relative_velocity(0) * air_relative_velocity(2);
        double j23 = air_relative_velocity(1) * air_relative_velocity(2);

        Eigen::Matrix3d vec_derivative;
        vec_derivative << j11, j12, j13,
                          j12, j22, j23,
                          j13, j23, j33;

        return vec_derivative / norm;
    }
    else
        return Eigen::Matrix3d::Zero();
}

WindVelocityObserver::~WindVelocityObserver()
{

}
