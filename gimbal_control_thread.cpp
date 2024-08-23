#include <chrono>
#include <algorithm>
#include "Log.h"
#include "pan_tilt_controller.h"
#include "adxl345_controller.h"
#include "gimbal_control_thread.h"

using std::get;

//-----------------------------------------------------------------------------
GimbalControlThread::GimbalControlThread(
   PanTiltController& pan_tilt,
   Adxl345Controller& imu )
   : coral::thread::IThread("gimbal_control_thread")
   , pan_tilt_( pan_tilt )
   , imu_( imu)
   , limit_x_min_(std::tuple<int16_t, bool>{0, false})
   , limit_x_max_(std::tuple<int16_t, bool>{0, false})
   , limit_y_min_(std::tuple<int16_t, bool>{0, false})
   , limit_y_max_(std::tuple<int16_t, bool>{0, false})
   , control_enabled_( false )
   , m_x_(0.0)
   , m_y_(0.0)
   , b_x_(0.0)
   , b_y_(0.0)
{
}

//-----------------------------------------------------------------------------
bool GimbalControlThread::all_limits_set() const
{
   bool limits_set = true;

   limits_set_ &= std::get<1>(limit_x_min_);
   limits_set_ &= std::get<1>(limit_x_max_);
   limits_set_ &= std::get<1>(limit_y_min_);
   limits_set_ &= std::get<1>(limit_y_min_);

   return limits_set;
}

//-----------------------------------------------------------------------------
bool GimbalControlThread::enable_tracking()
{
   if (all_limits_set())
   {
      control_enabled_ = true;
      return true;
   }

   return false;
}

//-----------------------------------------------------------------------------
void GimbalControlThread::set_lim_x_min( int16_t x_min )
{
   limit_x_min_ = Limit{x_min, true};
   compute_coefficients();
}

//-----------------------------------------------------------------------------
void GimbalControlThread::get_lim_x_min()
{
   return std::get<0>(limit_x_min_);
}

//-----------------------------------------------------------------------------
void GimbalControlThread::set_lim_x_max( int16_t x_max )
{
   limit_x_max_ = Limit{x_max, true};
   compute_coefficients();
}

//-----------------------------------------------------------------------------
void GimbalControlThread::get_lim_x_max()
{
   return std::get<0>(limit_x_max_);
}

//-----------------------------------------------------------------------------
void GimbalControlThread::set_lim_y_min( int16_t y_min )
{
   limit_y_min_ = Limit{y_min, true};
   compute_coefficients();
}

//-----------------------------------------------------------------------------
void GimbalControlThread::get_lim_y_min()
{
   return std::get<0>(limit_y_min_);
}

//-----------------------------------------------------------------------------
void GimbalControlThread::set_lim_y_max( int16_t y_max )
{
   limit_y_max_ = Limit{y_max, true};
   compute_coefficients();
}

//-----------------------------------------------------------------------------
void GimbalControlThread::get_lim_y_max()
{
   return std::get<0>(limit_y_max_);
}

//-----------------------------------------------------------------------------
void GimbalControlThread::compute_pan_tilt(
   const Adxl345Controller::AccelerationData& input,
   float& phi, float& theta )
{   
   float x = std::max(std::min(input.x, std::get<0>(limit_x_max_)), std::get<0>(limit_x_min_));
   float y = std::max(std::min(input.y, std::get<0>(limit_y_max_)), std::get<0>(limit_y_min_));
   phi   = m_x_ * x + b_x_;
   theta = m_y_ * y + b_y_;
}

//-----------------------------------------------------------------------------
void GimbalControlThread::compute_coefficients()
{
   if (all_limits_set())
   {
      m_y_ =
         (pan_tilt_.get_theta_max() - pan_tilt_.get_theta_min()) /
         static_cast<float>(std::get<0>(limit_y_max_) - std::get<0>(limit_y_min_));

      m_x_ =
         (pan_tilt_.get_phi_max() - pan_tilt_.get_phi_min()) /
         static_cast<float>(std::get<0>(limit_x_max_) - std::get<0>(limit_x_min_));

      b_y_ = pan_tilt_.get_theta_min();
      b_x_ = pan_tilt_.get_phi_min();
   }
}

//-----------------------------------------------------------------------------
void GimbalControlThread::run( const bool& shutdown )
{
   Adxl345Controller::AccelerationData accel_space;
   AngularVector pan_tilt_space;

   while ( shutdown == false )
   {
      const auto start{std::chrono::steady_clock::now()};

      if ( imu_.read_acceleration_data(accel_space) )
      {
         // coral::log::status("Accel: X=%d, Y=%d, Z=%d\n", vector.x, vector.y, vector.z);

         if ( control_enabled_ )
         {
            compute_pan_tilt( accel_space, pan_tilt_space );
            pan_tilt_.ease_position( pan_tilt_space.phi, pan_tilt_space.theta );
         }
      }

      const auto end{std::chrono::steady_clock::now()};
      int16_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      int32_t sleep_time_ms = kControlPeriodMs - elapsed_ms;
      if (sleep_time_ms < 0)
      {
         sleep_time_ms = 10;
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds( sleep_time_ms ));
   }
}
