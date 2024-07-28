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
// Date: 2015-12-05
// 

#include "Log.h"
#include "pwm_controller.h"
#include "pan_tilt_controller.h"

using namespace coral;

#define  PTC_MSEC_PER_SEC     ((float) 1000.0 )

const float PanTiltController::kMinAngleRadians;
const float PanTiltController::kMaxAngleRadians;

template<typename T>
bool in_range( const T& value, const T& min, const T& max )
{
   return ( ( value >= min ) && ( value <= max ) );
}

//-----------------------------------------------------------------------------
PanTiltController::PanTiltController(
   PwmController* controller,
   uint8_t pan_channel,
   uint8_t tilt_channel
)
   : controller_( controller )
   , pan_channel_( pan_channel )
   , tilt_channel_( tilt_channel )
   , theta_interp_slope_( 0.0 )
   , phi_interp_slope_( 0.0 )
   , theta_min_ticks_( 0 )
   , theta_max_ticks_( 0 )
   , phi_min_ticks_( 0 )
   , phi_max_ticks_( 0 )
   , current_phi_( 0.0 )
   , current_theta_( 0.0 )
{
   current_phi_ = (
      PanTiltController::kPhiMaxPulseWidthMs -
      PanTiltController::kPhiMinPulseWidthMs ) / 2.0;
   current_theta_ = (
      PanTiltController::kPhiMaxPulseWidthMs -
      PanTiltController::kPhiMinPulseWidthMs ) / 2.0;

   if ( controller_ )
   {
      float frequency = controller->get_frequency();

      theta_min_ticks_ = ( PwmController::kNumTicks * kThetaMinPulseWidthMs ) /
                   ( PTC_MSEC_PER_SEC / frequency );
      theta_max_ticks_ = ( PwmController::kNumTicks * kThetaMaxPulseWidthMs ) /
                   ( PTC_MSEC_PER_SEC / frequency );

      phi_min_ticks_ = ( PwmController::kNumTicks * kPhiMinPulseWidthMs ) /
                   ( PTC_MSEC_PER_SEC / frequency );
      phi_max_ticks_ = ( PwmController::kNumTicks * kPhiMaxPulseWidthMs ) /
                   ( PTC_MSEC_PER_SEC / frequency );

      phi_interp_slope_ = ( phi_max_ticks_ - phi_min_ticks_ ) /
                      ( kMaxAngleRadians - kMinAngleRadians );
      theta_interp_slope_ = ( theta_max_ticks_ - theta_min_ticks_ ) /
                      ( kMaxAngleRadians - kMinAngleRadians );

      set_position( current_phi_, current_theta_ );
   }
}

//-----------------------------------------------------------------------------
bool PanTiltController::set_position( float phi, float theta )
{
   bool success = false;

   if ( controller_ )
   {
      if ( in_range( phi, kMinAngleRadians, kMaxAngleRadians ) == false )
      {
         log::error(
            "PanTiltController::set_position: "
            "phi=%0.4f must be in range [%0.1f,%0.4f]\n",
            phi, kMinAngleRadians, kMaxAngleRadians );
      }
      else if ( in_range( theta, kMinAngleRadians, kMaxAngleRadians ) == false )
      {
         log::error(
            "PanTiltController::set_position: "
            "theta=%0.4f must be in range [%0.1f,%0.4f]\n",
            theta, kMinAngleRadians, kMaxAngleRadians );
      }
      else
      {
         uint16_t phi_ticks   = phi_interp_to_ticks( phi );
         uint16_t theta_ticks = theta_interp_to_ticks( theta );

         success  = controller_->set_pwm( pan_channel_, 0, phi_ticks );
         success &= controller_->set_pwm( tilt_channel_, 0, theta_ticks );

         if ( success )
         {
            current_phi_ = phi;
            current_theta_ = theta;
         }
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
float PanTiltController::get_phi() const
{
   return current_phi_;
}

//-----------------------------------------------------------------------------
float PanTiltController::get_theta() const
{
   return current_theta_;
}

//-----------------------------------------------------------------------------
uint16_t PanTiltController::phi_interp_to_ticks( const float& angle )
{
   return phi_interp_slope_ * angle + phi_min_ticks_;
}

//-----------------------------------------------------------------------------
uint16_t PanTiltController::theta_interp_to_ticks( const float& angle )
{
   return theta_interp_slope_ * angle + theta_min_ticks_;
}
