//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#ifndef _PIDLoop_h
#define _PIDLoop_h

//#include <TPixy2.h>

#define PID_MAX_INTEGRAL         100
#define ZUMO_BASE_DEADBAND       20

class PIDLoop{
public:
  PIDLoop();
  PIDLoop(int pgain, int igain, int dgain, bool servo, int minPos, int maxPos);
  void reset();
  void update(int error);
  int m_command;

private:
  int m_pgain;
  int m_igain;
  int m_dgain;

  int m_prevError;
  int m_integral;
  bool m_servo;
  int m_minPos;
  int m_maxPos;
};

#endif
