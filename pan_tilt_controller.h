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

class PwmController;

class PanTiltController {
public:

   PanTiltController( PwmController* controller, uint8_t pan_channel, uint8_t tilt_channel );

   static const float kMinAngleRadians = 0.0;
   static const float kMaxAngleRadians = 3.14159265358979323846;

   bool set_position( float phi, float theta );

   float get_phi() const;
   float get_theta() const;

private:

   uint16_t phi_interp_to_ticks( const float& angle );
   uint16_t theta_interp_to_ticks( const float& angle );

private:

   // @ 60Hz/4096 ticks: 45 = 403, 90 = 520
   static const float kThetaMinPulseWidthMs = 1.1637369791666667;
   static const float kThetaMaxPulseWidthMs = 3.068033854166667;

   // @ 60Hz/4096 ticks: 45 = 316, 90 = 431
   static const float kPhiMinPulseWidthMs = 0.8178710937500001;
   static const float kPhiMaxPulseWidthMs = 2.689615885416667;

   PwmController* controller_;

   uint8_t pan_channel_;
   uint8_t tilt_channel_;

   float theta_interp_slope_;
   float phi_interp_slope_;
   uint16_t theta_min_ticks_;
   uint16_t theta_max_ticks_;
   uint16_t phi_min_ticks_;
   uint16_t phi_max_ticks_;

   float current_phi_;
   float current_theta_;
};

#endif // PAN_TILT_CONTROLLER_H