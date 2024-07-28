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
// Date: 2015-11-15
// 

#ifndef  PWM_CONTROLLER_H
#define  PWM_CONTROLLER_H

#include <stdint.h>
#include <cstddef>

class I2cInterface;

class PwmController {
public:

   static const uint16_t kNumTicks = 4096;
   static const uint16_t kDefaultAddress = 0x0040;
   PwmController( I2cInterface* interface, uint16_t address = kDefaultAddress );

   bool initialize();

   bool set_frequency( uint16_t frequency_hz );
   int32_t get_frequency() const;

   bool set_pwm( size_t channel, uint16_t on_ticks, uint16_t off_ticks );

   bool set_all_pwm( uint16_t on_ticks, uint16_t off_ticks );

private:

   PwmController( const PwmController& );
   PwmController& operator= ( const PwmController& );

private:

   I2cInterface* i2c_;

   bool initialized_;

   uint16_t address_;

   int32_t frequency_hz_;

};

#endif // PWM_CONTROLLER_H
