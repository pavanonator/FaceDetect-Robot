#ifndef __CB_H__
#define __CB_H__

#include <armadillo>
#include "baserobot.h"

class CoffeeBot : public BaseRobot {
  public:
    CoffeeBot(void);
    ~CoffeeBot(void);
    int numconnected(void);
    void send(const arma::vec &motion); // [topleft topright botleft botright arm]
    arma::vec recv(void);
    void reset(void);

  private:
    arma::vec prev_motion;
    arma::vec motion_const;
};

#endif
