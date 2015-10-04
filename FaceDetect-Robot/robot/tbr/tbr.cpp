/****************************************
 *
 * The purpose of this program is to do 
 * the following for this particular bot:      
 *
 *  1) control the robot through
 *     abstracted methods
 *  2) send back sensor map values
 *
 ***************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <sys/types.h>
#include "tbr.h"

#define WHEEL_DEVID   1
#define ARM_DEVID     2
#define CLAW_DEVID    3
#define LEFT_SONAR    0
#define RIGHT_SONAR   1
#define BACK_SONAR    2
#define ARM_POT       0
#define MOT_LEFT      0
#define MOT_RIGHT     1
#define MOT_ARM       2
#define MOT_CLAW      3
#define WBUFSIZE      128

using namespace arma;

static double limitf(double x, double min, double max);

/** Constructor
 */
TennisBallRobot::TennisBallRobot(void) : BaseRobot(TENNIS_BALL_ROBOT) {
  this->prev_motion = zeros<vec>(4);
  this->motion_const = ones<vec>(4) * 255.0;
  this->sonar = zeros<vec>(3);
  this->pot = zeros<vec>(1);
  if (this->connect()) {
    this->reset();
    this->send(zeros<vec>(4));
  }
}

/** Destructor
 */
TennisBallRobot::~TennisBallRobot(void) {
  if (this->connected()) {
    this->send(zeros<vec>(4));
    this->reset();
    this->disconnect();
  }
}

/** Get the number of devices connected
 *  @return the number of devices that are connected
 */
int TennisBallRobot::numconnected(void) {
  return this->connections.size();
}

/** Reset the robot values
 */
void TennisBallRobot::reset(void) {
  this->prev_motion.zeros();
  this->sonar.zeros();
  this->pot.zeros();
}

/** Send output to the communication layer
 *  @param motion
 *    the motion vector
 */
void TennisBallRobot::send(const vec &motion) {
  vec new_motion = motion;
  // element match check
  if (new_motion.n_elem != motion_const.n_elem) {
    new_motion = zeros<vec>(motion_const.n_elem);
  }
  // software fix for the force feedback
  if (new_motion(MOT_LEFT) >= 0.0) {
    new_motion(MOT_LEFT) *= 1.15;
  }
  if (new_motion(MOT_RIGHT) >= 0.0) {
    new_motion(MOT_RIGHT) *= 1.15;
  }
  // boundary check
  for (int i = 0; i < (int)new_motion.n_elem; i++) {
    new_motion(i) = limitf(new_motion(i), -1.0, 1.0);
  }
  // amplitude factor
  new_motion %= motion_const;
  // send the values to the arduinos
  char msg[WBUFSIZE];
  for (int i = 0; i < (int)this->connections.size(); i++) {
    switch (this->ids[i]) {
      case WHEEL_DEVID:
        if (new_motion(MOT_LEFT)  == this->prev_motion(MOT_LEFT) &&
            new_motion(MOT_RIGHT) == this->prev_motion(MOT_RIGHT)) {
          break;
        } else {
          this->prev_motion(MOT_LEFT)  = new_motion(MOT_LEFT);
          this->prev_motion(MOT_RIGHT) = new_motion(MOT_RIGHT);
        }
        sprintf(msg, "[%d %d]\n",
            (int)new_motion(MOT_LEFT),
            (int)new_motion(MOT_RIGHT));
        serial_write(this->connections[i], msg);
        break;
      case ARM_DEVID:
        if (new_motion(MOT_ARM) == this->prev_motion(MOT_ARM)) {
          break;
        } else {
          this->prev_motion(MOT_ARM) = new_motion(MOT_ARM);
        }
        sprintf(msg, "[%d]\n",
            (int)new_motion(MOT_ARM));
        serial_write(this->connections[i], msg);
        break;
      case CLAW_DEVID:
        if (new_motion(MOT_CLAW) == this->prev_motion(MOT_CLAW)) {
          break;
        } else {
          this->prev_motion(MOT_CLAW) = new_motion(MOT_CLAW);
        }
        sprintf(msg, "[%d]\n",
            (int)new_motion(MOT_CLAW));
        serial_write(this->connections[i], msg);
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer
 */
vec TennisBallRobot::recv(void) {
  char *msg;
  int back_sonar;
  int left_sonar;
  int right_sonar;
  int arm_pot;
  for (int i = 0; i < (int)this->connections.size(); i++) {
    switch (this->ids[i]) {
      case WHEEL_DEVID:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d]\n", &this->ids[i],
            &back_sonar);
        this->sonar(BACK_SONAR) = (double)back_sonar;
        break;
      case ARM_DEVID:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &this->ids[i],
            &left_sonar, &right_sonar);
        this->sonar(LEFT_SONAR) = (double)left_sonar;
        this->sonar(RIGHT_SONAR) = (double)right_sonar;
        break;
      case CLAW_DEVID:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d]\n", &this->ids[i],
            &arm_pot);
        this->pot(ARM_POT) = (double)arm_pot;
        break;
      default:
        break;
    }
  }
  // adjust for sonar blockage by the arm
  if (this->pot[ARM_POT] < 650) {
    this->sonar[LEFT_SONAR] = 200;
    this->sonar[RIGHT_SONAR] = 200;
  }
  return vec({
      this->sonar(LEFT_SONAR),
      this->sonar(RIGHT_SONAR),
      this->sonar(BACK_SONAR),
      this->pot(ARM_POT) });
}

/** Limit a value between min and max
 *  @param x
 *    the value
 *  @param min
 *    the lower bound
 *  @param max
 *    the upper bound
 *  @return the limited value
 */
static double limitf(double x, double min, double max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}
