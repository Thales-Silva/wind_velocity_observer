#ifndef WIND_VELOCITY_OBSERVER_HPP
#define WIND_VELOCITY_OBSERVER_HPP

#include <iostream>

#include <cmath>

#include <boost/chrono.hpp>

#include <eigen3/Eigen/Dense>

#include <vector>

class WindVelocityObserver
{
    private:
        Eigen::Vector3d psi;

        double g;
        boost::chrono::steady_clock::time_point last_sample;

    protected:
        double mass;
        double mass_inverse;

        double thrust_coefficient;
        double rotor_drag_coefficient;

        double c;
        double inversion_gain;
        double gamma;

        Eigen::Matrix3d effort_coeffiecients;
        Eigen::Matrix3d effort_coeffiecients_inv;

        Eigen::Vector3d wind_velocity;
        Eigen::Vector3d air_relative_velocity;

    public:
        WindVelocityObserver();
        ~WindVelocityObserver();

        void updateObserver(Eigen::Quaterniond orientation,
                            Eigen::Vector3d linear_velocity,
                            std::vector<double> motor_speed);

        void getThrustAndRotorDrag(std::vector<double> spin_rate,
                                   Eigen::Matrix3d rotation_matrix,
                                   Eigen::Vector3d &thrust,
                                   Eigen::Matrix3d &rotor_drag_matrix);

        Eigen::Matrix3d evaluateVectorDerivative();
        inline Eigen::Matrix3d hatOperator(Eigen::Vector3d v)
        {
            Eigen::Matrix3d v_hat;
            v_hat <<   0, -v(2),  v(1),
                    v(2),     0, -v(0),
                   -v(1),  v(0),     0;
            return v_hat;
        }

        static const double STANDARD_GRAVITY;
};

#endif // WIND_VELOCITY_OBSERVER_HPP
