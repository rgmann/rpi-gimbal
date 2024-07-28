// 
// Copyright (c) 2015, Robert Glissmann
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 

// %% license-end-token %%
// 
// Author: Robert.Glissmann@gmail.com (Robert Glissmann)
// Date: 2015-11-28
// 


#define  ONE_DEGREE     ((float) 0.017453292519943295 )
#define  MOTION_STEP    ( ONE_DEGREE / 2.0 )

#define  PHI_HIGH_LIMIT_RAD   ((float) 2.9 )
#define  PHI_LOW_LIMIT_RAD    ((float) 0.2 )
#define  THETA_HIGH_LIMIT_RAD ((float) 2.9 )
#define  THETA_LOW_LIMIT_RAD  ((float) 0.2 )

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "Log.h"
#include "pan_tilt_controller.h"
#include "lidar_lite.h"
#include "pan_tilt_thread.h"

using namespace coral;
using namespace coral::thread;

//-----------------------------------------------------------------------------
PanTiltThread::PanTiltThread(
   PanTiltController& controller,
   LidarLite& lidar
)
   : IThread( "pan_tilt_thread" )
   , controller_( controller )
   , lidar_( lidar )
   , mode_( PanTiltThread::kIdle )
   , callback_ptr_( NULL )
   , measure_duration_ms_( 100 )
   , phi_raster_direction_( PanTiltThread::kIncreasing )
   , theta_raster_direction_( PanTiltThread::kIncreasing )
   , stare_phi_( 0.0 )
   , stare_theta_( 0.0 )
   , min_phi_( PHI_LOW_LIMIT_RAD )
   , max_phi_( PHI_HIGH_LIMIT_RAD )
   , min_theta_( THETA_LOW_LIMIT_RAD )
   , max_theta_( THETA_HIGH_LIMIT_RAD )
   , increment_( MOTION_STEP )
	, ll_eye_safe_( false )
	, ll_good_( false )
	, ll_status_( 0 )
{
   stare_phi_ = (
      PanTiltController::kMaxAngleRadians -
      PanTiltController::kMinAngleRadians ) / 2.0;
   stare_theta_ = stare_phi_;

   if ( controller_.set_position( stare_phi_, stare_theta_ ) == false )
   {
      log::error("Failed to initialize PWM position.\n");
   }
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_mode( ControlMode mode )
{
   mode_ = mode;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_min_phi( float min_phi )
{
   min_phi_ = min_phi;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_max_phi( float max_phi )
{
   max_phi_ = max_phi;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_min_theta( float min_theta )
{
   min_theta_ = min_theta;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_max_theta( float max_theta )
{
   max_theta_ = max_theta;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_increment( float increment )
{
   increment_ = increment;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_callback( MeasurementCallback* callback_ptr )
{
   boost::mutex::scoped_lock guard( callback_lock_ );
   callback_ptr_ = callback_ptr;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_hover_time( uint32_t duration_ms )
{
   measure_duration_ms_ = duration_ms;
}

//-----------------------------------------------------------------------------
void PanTiltThread::set_stare_point( float phi, float theta )
{
   stare_phi_   = phi;
   stare_theta_ = theta;
   mode_ = kStare;
}

//-----------------------------------------------------------------------------
void PanTiltThread::run( const bool& shutdown )
{
   PanTiltThread::Point point;

   while ( shutdown == false )
   {
      if ( mode_ == kRaster )
      {
         bool  newline = false;
         float phi     = controller_.get_phi();
         float theta   = controller_.get_theta();

         if ( phi_raster_direction_ == kIncreasing )
         {
            phi += increment_;

            if ( phi >= max_phi_ )
            {
               phi_raster_direction_ = kDecreasing;
               newline = true;
            }
         }
         else if ( phi_raster_direction_ == kDecreasing )
         {
            phi -= increment_;

            if ( phi <= min_phi_ )
            {
               phi_raster_direction_ = kIncreasing;
               newline = true;
            }
         }

         if ( newline )
         {
            if ( theta >= max_theta_ )
            {
               theta_raster_direction_ = kDecreasing;
            }
            else if ( theta <= min_theta_ )
            {
               theta_raster_direction_ = kIncreasing;
            }

            if ( theta_raster_direction_ == kIncreasing ) theta += increment_;
            else theta -= increment_;
         }

         controller_.set_position( phi, theta );

         point.phi   = phi;
         point.theta = theta;
      }
      else if ( mode_ == kStare )
      {
         controller_.set_position( stare_phi_, stare_theta_ );

         point.phi   = stare_phi_;
         point.theta = stare_theta_;
      }

		ll_eye_safe_ = lidar_.eye_safe();
      ll_good_ = lidar_.good();
      ll_status_ = lidar_.read_health_status();

      boost::this_thread::sleep(boost::posix_time::milliseconds( measure_duration_ms_ ));


      if ( callback_ptr_ && ( ( point.r = lidar_.get_range() ) >= 0 ) )
      {
         (*callback_ptr_)( mode_, point );
      }
   }
}
