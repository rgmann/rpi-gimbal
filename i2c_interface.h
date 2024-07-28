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

#ifndef  I2C_INTERFACE_H
#define  I2C_INTERFACE_H

#include <stdint.h>
#include <cstddef>

class I2cInterface {
public:

   enum i2c_error {
      kSuccess,
      kOpenError,
      kWriteError,
      kReadError,
      kInvalidAddessMode,
      kInvalidDeviceID,
      kInvalidDeviceHandle
   };

   ///
   /// Destructor
   ///
   ~I2cInterface();

   ///
   /// Get a pointer to the I2C interface instance
   ///
   /// @return I2cInterface*  Pointer to I2C interface instance
   ///
   static I2cInterface* instance( const char* device_path = NULL );
   static void destroy();

   ///
   /// Open the I2C interface
   ///
   /// @return bool True on success; false on failure
   ///
   bool open( const char* device_path );

   ///
   /// Close the I2C interface
   ///
   void close();

   ///
   /// 
   ///
   bool is_open() const;

   i2c_error acquire( uint16_t device_id );

   i2c_error read( void* buffer, size_t max_bytes, size_t& bytes_recvd );
   i2c_error read( uint8_t address, void* buffer, size_t max_bytes, size_t& bytes_recvd );

   i2c_error write( const void* buffer, size_t buffer_size );
   i2c_error write( uint8_t address, const void* buffer, size_t buffer_size );

private:

   I2cInterface();
   I2cInterface( const I2cInterface& ) {};
   I2cInterface& operator= ( const I2cInterface& ) {};

private:

   static const int kInvalidHandle = -1;
   
   int handle_;

   uint32_t capabilities_;

   static I2cInterface*  our_instance_;
};

#endif // I2C_INTERFACE_H
