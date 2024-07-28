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

#include <boost/thread/mutex.hpp>
#include "IThread.h"

#ifndef  PAN_TILT_THREAD_H
#define  PAN_TILT_THREAD_H

class PanTiltController;
class LidarLite;

class PanTiltThread : public coral::thread::IThread {
public:

   struct Point {
      float phi;
      float theta;
      float r;
   };
   enum ControlMode {
      kIdle,
      kRaster,
      kStare
   };

   class MeasurementCallback {
   public:

      virtual void operator()( ControlMode mode, const Point& point ) = 0;
   };

   PanTiltThread( PanTiltController& controller, LidarLite& lidar );


   void set_mode( ControlMode mode );
	ControlMode get_mode() const { mode_; }

   void set_callback( MeasurementCallback* callback_ptr );

   void set_hover_time( uint32_t duration_ms );

   void set_stare_point( float phi, float theta );
   float get_phi() const { return stare_phi_; }
   float get_theta() const { return stare_theta_; }

   void set_min_phi( float min_phi );
   float get_min_phi() const { return min_phi_; }
   void set_max_phi( float max_phi );
   float get_max_phi() const { return max_phi_; }
   void set_min_theta( float min_theta );
   float get_min_theta() const { return min_theta_; }
   void set_max_theta( float max_theta );
   float get_max_theta() const { return max_theta_; }

   void set_increment( float increment );

	bool get_ll_eye_safe() { return ll_eye_safe_; }
	bool get_ll_good() { return ll_good_; }
	uint8_t get_ll_status() { return ll_status_; }


private:

   void run( const bool& shutdown );

private:

   PanTiltController& controller_;
   LidarLite& lidar_;

   ControlMode mode_;
   uint32_t measure_duration_ms_;

   boost::mutex callback_lock_;
   MeasurementCallback* callback_ptr_;

   enum Direction {
      kIncreasing,
      kDecreasing
   };
   Direction phi_raster_direction_;
   Direction theta_raster_direction_;

   float stare_phi_;
   float stare_theta_;

   float min_phi_;
   float max_phi_;
   float min_theta_;
   float max_theta_;
   float increment_;

	bool ll_eye_safe_;
	bool ll_good_;
	uint8_t ll_status_;
};

#endif // PAN_TILT_THREAD_H
