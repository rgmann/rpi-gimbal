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

   bool enable_tracking();
   void disable_tracking() { control_enabled_ = false; }

   bool all_limits_set() const;

   void set_limit_x_min(int16_t);
   int16_t get_limit_x_min() const;
   void set_limit_x_max(int16_t);
   int16_t get_limit_x_max() const;

   void set_limit_y_min(int16_t);
   int16_t get_limit_y_min() const;
   void set_limit_y_max(int16_t);
   int16_t get_limit_y_max() const;

private:

   void compute_pan_tilt(
      const Adxl345Controller::AccelerationData& input,
      float& phi,
      float& theta );

   void compute_coefficients();

   void run( const bool& shutdown );

private:

   PanTiltController& pan_tilt_;

   Adxl345Controller& imu_;

   typedef Limit std::tuple<int16_t, bool>;
   Limit limit_x_min_;
   Limit limit_x_max_;
   Limit limit_y_min_;
   Limit limit_y_max_;

   bool control_enabled_;

   float m_x_;
   float m_y_;

   float b_x_;
   float b_y_;

   static constexpr int32_t kControlPeriodMs = 100;
};

#endif // GIMBAL_CONTROL_THREAD_H