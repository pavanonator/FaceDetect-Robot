#include "tachikoma.h"
#include "xboxctrl.h"
#include "defs.h"
#include <iostream>
#include <signal.h>

using namespace arma;
static bool stopsig;

void stopsignal(int) {
  stopsig = true;
}

int main() {
  signal(SIGINT, stopsignal);
  xboxctrl_t ctrl;
  // connect to the tachikoma
  Tachikoma tachikoma;
  // get the controller
  xboxctrl_connect(&ctrl);
  
  if (!tachikoma.connected() ||
      !ctrl.connected ||
      tachikoma.numconnected() < 4) {
    tachikoma.disconnect();
    xboxctrl_disconnect(&ctrl);
    return 1;
  }

  int leg_select = 0;
  mat leg_vel(NUM_JOINTS, NUM_LEGS);
  vec wheels(NUM_LEGS);
  while (!stopsig) {
    // handle the controller events
    xboxctrl_update(&ctrl);
    if (ctrl.Y) {
      leg_select = UL;
      leg_vel.zeros();
    } else if (ctrl.X) {
      leg_select = DL;
      leg_vel.zeros();
    } else if (ctrl.B) {
      leg_select = UR;
      leg_vel.zeros();
    } else if (ctrl.A) {
      leg_select = DR;
      leg_vel.zeros();
    }
    // assign the values from the controller to the value storage
    leg_vel(WAIST, leg_select) = (double)(ctrl.RIGHT - ctrl.LEFT);
    leg_vel(THIGH, leg_select) = ctrl.LJOY.y;
    leg_vel(KNEE, leg_select) = ctrl.RJOY.y;
    wheels(leg_select) = (double)(ctrl.UP - ctrl.DOWN);
    // send over the values to the robot
    tachikoma.send(zeros<mat>(NUM_JOINTS, NUM_LEGS), leg_vel, wheels,
                   zeros<mat>(1, 1), false, true);
    // print out the feedback from the robot
    mat leg_sensors;
    mat leg_feedback;
    tachikoma.recv(leg_sensors, leg_feedback);
    std::cout << leg_feedback << std::endl;
  }

  return 0;
}
