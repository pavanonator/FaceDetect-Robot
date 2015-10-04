#ifndef __TK_TBR_H__
#define __TK_TBR_H__

#include <armadillo>
#include "baserobot.h"

class TennisBallRobot : public BaseRobot {
  public:
    TennisBallRobot(void);
    ~TennisBallRobot(void);
    int numconnected(void);
    void send(const arma::vec &motion);
    arma::vec recv(void);
    void reset(void);

  private:
    arma::vec prev_motion;
    arma::vec motion_const;
    arma::vec sonar;
    arma::vec pot;
};

#endif
