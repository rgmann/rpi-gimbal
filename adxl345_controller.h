
#ifndef ADXL345_INTERFACE_H
#define ADXL345_INTERFACE_H

#include <stdint.h>
#include <cstddef>

class I2cInterface;

class Adxl345Controller {
public:

   static constexpr uint16_t kDeviceAddress = 0x53;

   Adxl345Controller( I2cInterface&, uint16_t address = kDeviceAddress );

   bool initialize();

   enum AxisType {
      XAxis = 0,
      YAxis = 1,
      ZAxis = 2,
      NumAxis
   };

   enum RangeSetting {
      kRange2g,
      kRange4g,
      kRange8g,
      kRange16g
   };
   bool set_range_setting(RangeSetting setting);

   struct AccelerationData {
      int16_t x;
      int16_t y;
      int16_t z;
   };

   bool read_acceleration_data(AccelerationData& data);

private:

   Adxl345Controller( const Adxl345Controller& );
   Adxl345Controller& operator= ( const Adxl345Controller& );

private:

   I2cInterface& i2c_;

   bool initialized_;

   uint16_t address_;

   uint8_t last_error_;

   double gains_[AxisType::NumAxis];
};


#endif // ADXL345_INTERFACE_H