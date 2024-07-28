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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "Log.h"
#include "i2c_interface.h"
#include "pwm_controller.h"

using namespace coral;

// Registers/etc.
#define PWMC_REG_MODE1               0x00
#define PWMC_REG_MODE2               0x01
#define PWMC_REG_SUBADR1             0x02
#define PWMC_REG_SUBADR2             0x03
#define PWMC_REG_SUBADR3             0x04
#define PWMC_REG_PRESCALE            0xFE
#define PWMC_REG_LED0_ON_L           0x06
#define PWMC_REG_LED0_ON_H           0x07
#define PWMC_REG_LED0_OFF_L          0x08
#define PWMC_REG_LED0_OFF_H          0x09
#define PWMC_REG_ALL_LED_ON_L        0xFA
#define PWMC_REG_ALL_LED_ON_H        0xFB
#define PWMC_REG_ALL_LED_OFF_L       0xFC
#define PWMC_REG_ALL_LED_OFF_H       0xFD

// Bits
#define PWMC_RESTART                0x80
#define PWMC_SLEEP                  0x10
#define PWMC_ALLCALL                0x01
#define PWMC_INVRT                  0x10
#define PWMC_OUTDRV                 0x04


#define PWMC_25MHZ               25000000.0


//-----------------------------------------------------------------------------
PwmController::PwmController( I2cInterface* interface, uint16_t address )
   : i2c_( interface )
   , address_( address )
   , initialized_( false )
   , frequency_hz_( -1 )
{
}

//-----------------------------------------------------------------------------
bool PwmController::initialize()
{
   bool success = set_all_pwm( 0, 0 );

   if ( success )
   {
      uint8_t mode2 = PWMC_OUTDRV;

      if ( i2c_->write(
         PWMC_REG_MODE2,
         &mode2,
         sizeof( mode2 )
      ) == I2cInterface::kSuccess )
      {
         uint8_t mode1 = PWMC_ALLCALL;

         if ( i2c_->write(
            PWMC_REG_MODE1,
            &mode1,
            sizeof( mode1 )
         ) == I2cInterface::kSuccess )
         {
            // TODO: Sleep 5 ms
            boost::this_thread::sleep(boost::posix_time::milliseconds(5));

            uint8_t current_mode = 0;
            size_t bytes_rcvd = 0;

            if ( ( i2c_->read(
                  PWMC_REG_MODE1,
                  &current_mode,
                  sizeof( current_mode ),
                  bytes_rcvd
               ) == I2cInterface::kSuccess ) &&
               ( bytes_rcvd == sizeof( current_mode ) )
            )
            {
               log::status("initialize: CURRENT MODE = 0x%02X\n", current_mode);
               // Wake the device by clearing the sleep bit.
               current_mode &= ~PWMC_SLEEP;

               if ( i2c_->write(
                  PWMC_REG_MODE1,
                  &current_mode,
                  sizeof( current_mode )
               ) == I2cInterface::kSuccess )
               {
                  boost::this_thread::sleep(boost::posix_time::milliseconds(5));
                  success = true;
                  initialized_ = true;
               }
               else
               {
                  log::error("PwmController::initialize: Failed to set current mode\n");
                  success = false;
               }
            }
            else
            {
               log::error("PwmController::initialize: Failed to read current mode\n");
               success = false;
            }
         }
         else
         {
            log::error("PwmController::initialize: Failed to set current mode 1\n");
            success = false;
         }
      }
      else
      {
         log::error("PwmController::initialize: Failed to set current mode 2\n");
         success = false;
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
bool PwmController::set_frequency( uint16_t frequency )
{
   bool success = false;

   if ( i2c_ && initialized_ )
   {
      if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
      {
         double prescale_value = ( PWMC_25MHZ / 4096.0 ) / (double)frequency;
         prescale_value -= 1.0;

         uint8_t prescale = floor( prescale_value + 0.5 );

         size_t bytes_rcvd = 0;
         uint8_t orignal_mode = 0;

         if ( i2c_->read( PWMC_REG_MODE1, &orignal_mode, sizeof( orignal_mode ), bytes_rcvd ) == I2cInterface::kSuccess )
         {
            uint8_t new_mode = ( orignal_mode & 0x7F ) | PWMC_SLEEP;
            log::status("CURRENT MODE = 0x%02X, new_mode=%02X\n", orignal_mode, new_mode);

            if ( i2c_->write( PWMC_REG_MODE1, &new_mode, sizeof( new_mode ) ) == I2cInterface::kSuccess )
            {
               if ( i2c_->write( PWMC_REG_PRESCALE, &prescale, sizeof( prescale ) ) != I2cInterface::kSuccess )
               {
                  log::error("PwmController::set_frequency: Failed to set prescale of %u.\n", prescale);
                  success = false;
               }

               // Regardless of whether the prescaler was successfully set, return
               // to the old mode.
               if ( i2c_->write( PWMC_REG_MODE1, &orignal_mode, sizeof( orignal_mode ) ) == I2cInterface::kSuccess )
               {
                  // TODO: Sleep 5 ms
                  boost::this_thread::sleep(boost::posix_time::milliseconds(5));

                  orignal_mode |= 0x80;
                  if ( i2c_->write( PWMC_REG_MODE1, &orignal_mode, sizeof( orignal_mode ) ) == I2cInterface::kSuccess )
                  {
                     frequency_hz_ = (int32_t)frequency;
                     success = true;
                  }
                  else
                  {
                     log::error("PwmController::set_frequency: ERROR at %d\n",__LINE__);
                     success = false;
                  }
               }
               else
               {
                  log::error("PwmController::set_frequency: ERROR at %d\n",__LINE__);
                  success = false;
               }
            }
            else
            {
               log::error("PwmController::set_frequency: ERROR at %d\n",__LINE__);
               success = false;
            }
         }
         else
         {
            log::error("PwmController::set_frequency: ERROR at %d\n",__LINE__);
            success = false;
         }
      }
      else
      {
         log::error("PwmController::set_frequency: ERROR at %d\n",__LINE__);
         success = false;
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
int32_t PwmController::get_frequency() const
{
   return frequency_hz_;
}

//-----------------------------------------------------------------------------
bool PwmController::set_pwm( size_t channel, uint16_t on_ticks, uint16_t off_ticks )
{
   bool success = false;

   if ( i2c_ && initialized_ )
   {
      if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
      {
         uint8_t on_ticks_lower = on_ticks & 0xFF;

         if ( i2c_->write(
            PWMC_REG_LED0_ON_L + 4 * channel,
            &on_ticks_lower,
            sizeof( on_ticks_lower )
         ) == I2cInterface::kSuccess )
         {
            uint8_t on_ticks_upper = on_ticks >> 8;

            if ( i2c_->write(
               PWMC_REG_LED0_ON_H + 4 * channel,
               &on_ticks_upper,
               sizeof( on_ticks_upper )
            ) == I2cInterface::kSuccess )
            {
               uint8_t off_ticks_lower = off_ticks & 0xFF;

               if ( i2c_->write(
                  PWMC_REG_LED0_OFF_L + 4 * channel,
                  &off_ticks_lower,
                  sizeof( off_ticks_lower )
               ) == I2cInterface::kSuccess )
               {
                  uint8_t off_ticks_upper = off_ticks >> 8;

                  if ( i2c_->write(
                     PWMC_REG_LED0_OFF_H + 4 * channel,
                     &off_ticks_upper,
                     sizeof( off_ticks_upper )
                  ) == I2cInterface::kSuccess )
                  {
                     success = true;
                  }
                  else
                  {
                     log::error("PwmController::set_pwm: ERROR at %d\n",__LINE__);
                     success = false;
                  }
               }
               else
               {
                  log::error("PwmController::set_pwm: ERROR at %d\n",__LINE__);
                  success = false;
               }
            }
            else
            {
               log::error("PwmController::set_pwm: ERROR at %d\n",__LINE__);
               success = false;
            }
         }
         else
         {
            log::error("PwmController::set_pwm: ERROR at %d\n",__LINE__);
            success = false;
         }
      }
      else
      {
         log::error("PwmController::set_pwm: ERROR at %d\n",__LINE__);
         success = false;
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
bool PwmController::set_all_pwm( uint16_t on_ticks, uint16_t off_ticks )
{
   bool success = false;

   if ( i2c_ )
   {
      if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
      {
         uint8_t on_ticks_lower = on_ticks & 0xFF;

         if ( i2c_->write(
            PWMC_REG_ALL_LED_ON_L,
            &on_ticks_lower,
            sizeof( on_ticks_lower )
         ) == I2cInterface::kSuccess )
         {
            uint8_t on_ticks_upper = on_ticks >> 8;

            if ( i2c_->write(
               PWMC_REG_ALL_LED_ON_H,
               &on_ticks_upper,
               sizeof( on_ticks_upper )
            ) == I2cInterface::kSuccess )
            {
               uint8_t off_ticks_lower = off_ticks & 0xFF;

               if ( i2c_->write(
                  PWMC_REG_ALL_LED_OFF_L,
                  &off_ticks_lower,
                  sizeof( off_ticks_lower )
               ) == I2cInterface::kSuccess )
               {
                  uint8_t off_ticks_upper = off_ticks >> 8;

                  if ( i2c_->write(
                     PWMC_REG_ALL_LED_OFF_H,
                     &off_ticks_upper,
                     sizeof( off_ticks_upper )
                  ) == I2cInterface::kSuccess )
                  {
                     success = true;
                  }
                  else
                  {
                     log::error("PwmController::set_all_pwm: ERROR at %d\n",__LINE__);
                     success = false;
                  }
               }
               else
               {
                  log::error("PwmController::set_all_pwm: ERROR at %d\n",__LINE__);
                  success = false;
               }
            }
            else
            {
               log::error("PwmController::set_all_pwm: ERROR at %d\n",__LINE__);
               success = false;
            }
         }
         else
         {
            log::error("PwmController::set_all_pwm: ERROR at %d\n",__LINE__);
            success = false;
         }
      }
      else
      {
         log::error("PwmController::set_all_pwm: ERROR at %d\n",__LINE__);
         success = false;
      }
   }

   return success;
}
