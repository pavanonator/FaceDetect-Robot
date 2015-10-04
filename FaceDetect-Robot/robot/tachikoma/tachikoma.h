#ifndef __TK_TACHIKOMA_H__
#define __TK_TACHIKOMA_H__

#include <armadillo>
#include "baserobot.h"

class Tachikoma : public BaseRobot {
  public:
    /** Constructor
     */
    Tachikoma(void);

    /** Deconstructor
     */
    ~Tachikoma(void);

    /** Detect if the tachikoma is connected to all ports or not
     *  @return true if all ports are connected, false otherwise
     */
    bool connected(void);

    /** Detect the number of devices that are connected to the Tachikoma
     *  @return the number of ports that are connected
     */
    int numconnected(void);

    /** Send vectors of values to the microcontrollers
     *  @param leg_theta a 3x4 matrix representing the motion positions
     *  @param leg_vel a 3x4 matrix representing the motion velocities
     *  @param wheels a 4x1 vector representing the wheel velocities
     *  @param arm_theta a ?x2 matrix representing the arm positions
     *  @param leg_theta_act (optional) a boolean representing the position enable
     *  @param leg_vel_act (optional) a boolean representing the velocity enable
     */
    void send(const arma::mat &leg_theta,
              const arma::mat &leg_vel,
              const arma::vec &wheels,
              const arma::mat &arm_theta,
              bool leg_theta_act = true,
              bool leg_vel_act = false);

    /** Receive a matrix of sensor values from the legs, indicating (for now) angles and touch
     *  @param leg_sensors a generic 4x4 matrix representing the theta and distances of the legs
     *  @param leg_feedback a generic 4x4 matrix representing the vectors of the motors
     *  @return for compatability, returns a vector of all sensor values
     */
    arma::vec recv(arma::mat &leg_sensors, arma::mat &leg_feedback);

    /** Reset the robot's default values
     */
    void reset(void);

    /** Solve the xyz coordinate of the leg using forward kinematics
     *  @param waist, thigh, knee
     *    the current encoder value vector (waist, thigh, knee)
     *  @param legid
     *    the id the leg to solve for
     *  @return the position vector (x, y, z)
     */
    arma::vec leg_fk_solve(const arma::vec &enc, int legid);

    /** Solve the encoder values of the legs given a target
     *  @param pos
     *    the target position vector (x, y, z)
     *  @param enc
     *    the current encoder value vector (waist, thigh, knee)
     *  @param legid
     *    the id the leg to solve for
     *  @return the differential encoder vector (dx, dy, dz)
     */
    arma::vec leg_ik_solve(const arma::vec &pos, const arma::vec &enc, int legid);

    // updated on send
    arma::mat leg_write;
    // updated on recv
    arma::mat leg_read;
    // updated on forward kinematics
    arma::mat leg_positions;

  private:
    char thigh_signature;
};

#endif
