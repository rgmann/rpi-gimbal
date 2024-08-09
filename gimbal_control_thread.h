#include <boost/thread/mutex.hpp>
#include "IThread.h"

#ifndef  GIMBAL_CONTROL_THREAD_H
#define  GIMBAL_CONTROL_THREAD_H

class PanTiltController;
class Adxl345Controller;

class GimbalControlThread : public coral::thread::IThread {
public:

    GimbalControlThread(PanTiltController&, Adxl345Controller&);

private:

    GimbalControlThread(const GimbalControlThread&);
    GimbalControlThread& operator= (const GimbalControlThread&);

private:

   void run( const bool& shutdown );

private:

    PanTiltController& pan_tilt_;

    Adxl345Controller& imu_;

    static constexpr int32_t kControlPeriodMs = 100;
};

#endif // GIMBAL_CONTROL_THREAD_H