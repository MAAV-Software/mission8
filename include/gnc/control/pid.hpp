#pragma once

namespace maav
{
namespace gnc
{
namespace control
{
/*
 * @brief The PID Controller
 *
 * @author Steven Schulte (spschul)
 *
 * @details The PID (Proportional-Integral-Derivative) class is used for
 * creating PID controllers. The PID controller sets its control variable based
 * off of the difference between the desired value and the actual value of some
 * input. It produces a new value for its control variable by scaling the error,
 * the change in error over time, and the accumulated error over time by
 * manually set gains.
 */
class Pid
{
    public:
    Pid();

    /*
     * @brief PID contructor
     *
     * @details The constructor for the PID class only requires the gains
     */
    Pid(double p, double i, double d);

    /*
     * @brief Set PID gains
     *
     * @details Set proportional, integral, and derivative gains.
     *
     * @param p New P gain
     * @param i New I gain
     * @param d New D gain
     */

    void setGains(double p, double i, double d);

    /*
     * @brief Return the control variable
     *
     * @details return what to set the control variable to based on the error
     * and the P, I, and D gains.
     *
     * @param e The error (the difference between the setpoint and the current
     * value of the variable)
     *
     * @param edot The change in error with respect to time
     */

    double run(double e, double edot);

    /*
     * @brief Get control variable with discrete derivative
     *
     * @details Like the run() function, but instead of supplying a derivative,
     * we calculate the error using the last error and a discrete derivative
     *
     * @param e The error (the difference between the setpoint and the current
     * value of the variable)
     *
     * @param dt The time that has elapsed since the PID controller was last run
     */
    double runDiscrete(double e, double dt);

    /*
        * @brief Reset the controller
        *
        * @details Reset the stored/accumulated error. Useful for things like
        * changing the PID gains mid-flight.
        */

    void reset();

    private:
    //! Proportional gain
    double _kp;

    //! Integral gain
    double _ki;

    //! Derivative gain
    double _kd;

    //! Accumulated sum of error
    double _eint;

    //! Last error received (for discrete derivatives)
    double _eprev;
};
}
}
}
