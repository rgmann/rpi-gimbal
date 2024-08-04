
#include <string>
#include <iostream>
#include <sstream>
#include "Log.h"
#include "InteractiveCommandRouter.h"

#include "i2c_interface.h"
#include "pwm_controller.h"
#include "pan_tilt_controller.h"
// #include "pan_tilt_thread.h"

using namespace coral;
using namespace coral::cli;

#define  PWM_FREQ_HZ    60

#define  PAN_CHANNEL    0
#define  TILT_CHANNEL   4

float rad_to_deg(float rad)
{
   return (rad * 180.0) / 3.14159;
}

float deg_to_rad(float deg)
{
return (deg * 3.14159) / 180.0;
}


class PanCommand : public InteractiveCommand {
public:
   PanCommand( PanTiltController& pan_tilt )
      : InteractiveCommand( "pan", "Pan gimbal")
      , pan_tilt_( pan_tilt ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      float new_theta = pan_tilt_.get_theta() + deg_to_rad(std::stof(args[0]));
      pan_tilt_.set_position( pan_tilt_.get_phi(), new_theta );
   }
private:

   PanTiltController& pan_tilt_;
};

class TiltCommand : public InteractiveCommand {
public:
   TiltCommand( PanTiltController& pan_tilt )
      : InteractiveCommand( "tilt", "Tilt gimbal")
      , pan_tilt_( pan_tilt ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      float new_phi = pan_tilt_.get_phi() + deg_to_rad(std::stof(args[0]));
      pan_tilt_.set_position( new_phi, pan_tilt_.get_theta() );
   }
private:

   PanTiltController& pan_tilt_;
};

class PointCommand : public InteractiveCommand {
public:
   PointCommand( PanTiltController& pan_tilt )
      : InteractiveCommand( "point", "Set position" )
      , pan_tilt_( pan_tilt ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      float new_theta = deg_to_rad(std::stof(args[0]));
      float new_phi = deg_to_rad(std::stof(args[1]));
      pan_tilt_.set_position( new_theta, new_phi );
   }
private:

   PanTiltController& pan_tilt_;
};

class GetPointCommand : public InteractiveCommand {
public:

   GetPointCommand( PanTiltController& pan_tilt )
      : InteractiveCommand( "getpoint", "Get position" )
      , pan_tilt_( pan_tilt ) {};

   void process(const coral::cli::ArgumentList& args)
   {
      std::stringstream position;
      position << "Theta = " << rad_to_deg(pan_tilt_.get_theta()) << "deg,  Phi = " << rad_to_deg(pan_tilt_.get_phi()) << " deg" << std::endl;
      coral::log::status(position.str().c_str());
   }
private:

   PanTiltController& pan_tilt_;
};


// class PointCallback : public PanTiltThread::MeasurementCallback {
// public:

//    void operator()( PanTiltThread::ControlMode mode, const PanTiltThread::Point& point )
//    {
//       if ( mode == PanTiltThread::kRaster )
//          log::status("phi = %0.4f, theta = %0.4f, r = %0.6f\n",point.phi,point.theta,point.r);
//    }

// };


int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   I2cInterface* i2c = I2cInterface::instance( "/dev/i2c-1" );

   if ( i2c )
   {
      PwmController    pwm( i2c );
      // Ads1115Interface adc( i2c );

      if ( pwm.initialize() )
      {
         bool init_success = true;

         if ( pwm.set_frequency( PWM_FREQ_HZ ) == false )
         {
            log::error("Failed to configure PWM frequency.\n");
            init_success = false;
         }

         if ( init_success )
         {
            PanTiltController pan_tilt( &pwm, PAN_CHANNEL, TILT_CHANNEL );

            InteractiveCommandRouter router;

            router.add( std::make_shared<PanCommand>(pan_tilt) );
            router.add( std::make_shared<TiltCommand>(pan_tilt) );
            router.add( std::make_shared<PointCommand>(pan_tilt) );
            router.add( std::make_shared<GetPointCommand>(pan_tilt) );
            
            router.run();
         }
         else
         {
            log::error("Failed to set PWM frequency.\n");
         }
      }
      else
      {
         log::error("Failed to initialize PWM controller.\n");
      }
   }
   else
   {
      log::error("Failed to open I2C interface.\n");
   }

   I2cInterface::destroy();

   log::flush();

   return 0;
}
