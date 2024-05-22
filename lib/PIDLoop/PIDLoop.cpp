#include <Arduino.h>
#include "PIDLoop.h"

PIDLoop::PIDLoop(){
  
}

PIDLoop::PIDLoop(int pgain, int igain, int dgain, bool servo, int minPos, int maxPos){
  m_pgain = pgain;
  m_igain = igain;
  m_dgain = dgain;
  m_servo = servo;
  m_minPos = minPos;
  m_maxPos = maxPos;

  reset();
}

void PIDLoop::reset(){
  if (m_servo)
  m_command = 90;
  else
  m_command = 0;

  m_integral = 0;
  m_prevError = 0x80000000L;
}

void PIDLoop::update(int error){
  {
    int pid;

    if (m_prevError!=0x80000000L)
    {
      // integrate integral
      m_integral += error;
      // bound the integral
      if (m_integral>PID_MAX_INTEGRAL)
      m_integral = PID_MAX_INTEGRAL;
      else if (m_integral<-PID_MAX_INTEGRAL)
      m_integral = -PID_MAX_INTEGRAL;

      // calculate PID term
      pid = (error*m_pgain + ((m_integral*m_igain)>>4) + (error - m_prevError)*m_dgain)>>10;

      if (m_servo)
      {
        m_command += pid; // since servo is a position device, we integrate the pid term
        if (m_command>m_maxPos)
        m_command = m_maxPos;
        else if (m_command<m_minPos)
        m_command = m_minPos;
      }
      else
      {
        // Deal with Zumo base deadband
        if (pid>0)
        pid += ZUMO_BASE_DEADBAND;
        else if (pid<0)
        pid -= ZUMO_BASE_DEADBAND;
        m_command = pid; // Zumo base is velocity device, use the pid term directly
      }
    }

    // retain the previous error val so we can calc the derivative
    m_prevError = error;
  }
};
