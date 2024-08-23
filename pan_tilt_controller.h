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

#ifndef  PAN_TILT_CONTROLLER_H
#define  PAN_TILT_CONTROLLER_H

#include <tuple>

class PwmController;

class PanTiltController {
public:

   PanTiltController( PwmController* controller, uint8_t pan_channel, uint8_t tilt_channel );

   static constexpr float kMinAngleRadians = 0.0;
   static constexpr float kMaxAngleRadians = 3.14159265358979323846;
   static constexpr float kMaxSpeedRadPerSec = 1.0;

   bool set_speed( float rad_per_sec );
   float get_speed() const;

   bool set_position( float phi_rad, float theta_rad );
   bool ease_position( float phi_rad, float theta_rad );

   float get_phi() const;
   float get_theta() const;

   void set_limit_phi_min(float rad);
   void set_limit_phi_max(float rad);

   float get_limits_phi_min() const;
   float get_limits_phi_max() const;

   void set_limit_theta_min(float rad);
   void set_limit_theta_max(float rad);

   float get_limits_theta_min() const;
   float get_limits_theta_max() const;

   bool all_limits_set() const;

private:

   uint16_t phi_interp_to_ticks( const float& angle );
   uint16_t theta_interp_to_ticks( const float& angle );

private:

   // @ 60Hz/4096 ticks: 45 = 403, 90 = 520
   static constexpr float kThetaMinPulseWidthMs = 1.1637369791666667;
   static constexpr float kThetaMaxPulseWidthMs = 3.068033854166667;

   // @ 60Hz/4096 ticks: 45 = 316, 90 = 431
   static constexpr float kPhiMinPulseWidthMs = 0.8178710937500001;
   static constexpr float kPhiMaxPulseWidthMs = 2.689615885416667;

   PwmController* controller_;

   uint8_t pan_channel_;
   uint8_t tilt_channel_;

   float theta_interp_slope_;
   float phi_interp_slope_;
   uint16_t theta_min_ticks_;
   uint16_t theta_max_ticks_;
   uint16_t phi_min_ticks_;
   uint16_t phi_max_ticks_;

   float speed_rad_per_sec_;

   float current_phi_;
   float current_theta_;

   typedef std::tuple<float, bool> Limit;

   Limit limit_phi_min_rad_;
   Limit limit_phi_max_rad_;

   Limit limit_theta_min_rad_;
   Limit limit_theta_max_rad_;
};

#endif // PAN_TILT_CONTROLLER_H