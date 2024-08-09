#include <chrono>
#include "Log.h"
#include "pan_tilt_controller.h"
#include "adxl345_controller.h"
#include "gimbal_control_thread.h"

//-----------------------------------------------------------------------------
GimbalControlThread::GimbalControlThread(
   PanTiltController& pan_tilt,
   Adxl345Controller& imu )
   : coral::thread::IThread("gimbal_control_thread")
   , pan_tilt_( pan_tilt )
   , imu_( imu)
{
}

//-----------------------------------------------------------------------------
void GimbalControlThread::run( const bool& shutdown )
{
   Adxl345Controller::AccelerationData vector;

   while ( shutdown == false )
   {
      const auto start{std::chrono::steady_clock::now()};

      if ( imu_.read_acceleration_data(vector) )
      {
         coral::log::status("Accel: X=%d, Y=%d, Z=%d\n", vector.x, vector.y, vector.z);
      }

      const auto end{std::chrono::steady_clock::now()};
      const std::chrono::duration<double> elapsed_seconds{end - start};
      int16_t elapsed_ms = std::chrono::milliseconds(elapsed_seconds).count();

      int32_t sleep_time_ms = kControlPeriodMs - elapsed_ms;
      if (sleep_time_ms < 0)
      {
         sleep_time_ms = 10;
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds( sleep_time_ms ));
   }
}
