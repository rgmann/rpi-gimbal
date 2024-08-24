
#include <string>
#include <iostream>
#include <sstream>
#include "Log.h"
#include "InteractiveCommandRouter.h"
#include "StringHelper.h"

#include "i2c_interface.h"
#include "pwm_controller.h"
#include "pan_tilt_controller.h"
#include "adxl345_controller.h"
#include "gimbal_control_thread.h"

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
      pan_tilt_.ease_position( pan_tilt_.get_phi(), new_theta );
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
      pan_tilt_.ease_position( new_phi, pan_tilt_.get_theta() );
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
      pan_tilt_.ease_position( new_theta, new_phi );
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

class SetSpeedCommand : public InteractiveCommand {
public:

   SetSpeedCommand( PanTiltController& pan_tilt )
      : InteractiveCommand( "setspeed", "Set speed" )
      , pan_tilt_( pan_tilt ) {};

   void process(const coral::cli::ArgumentList& args)
   {
      pan_tilt_.set_speed(deg_to_rad(std::stof(args[0])));
   }
private:

   PanTiltController& pan_tilt_;
};

class ReadImuCommand : public InteractiveCommand {
public:

   ReadImuCommand( Adxl345Controller& imu )
      : InteractiveCommand( "imu", "Read IMU" )
      , imu_( imu ) {};

   void process( const coral::cli::ArgumentList& args )
   {
      Adxl345Controller::AccelerationData vector;
      if ( imu_.read_acceleration_data(vector) )
      {
         coral::log::status("X=%d, Y=%d, Z=%d\n", vector.x, vector.y, vector.z);
      }
      else
      {
         coral::log::error("ERROR reading from IMU");
      }
   }

private:

   Adxl345Controller& imu_;
};

class CalibrateImuCommand : public InteractiveCommand {
public:

   CalibrateImuCommand( Adxl345Controller& imu, GimbalControlThread& control )
      : InteractiveCommand( "clim", "Calibrate IMU" )
      , imu_( imu )
      , control_( control )
   {
   }

   bool valid_limit_specifier(const std::string& limit_specifier)
   {
      if ( limit_specifier == "x_min" )
      {
         return true;
      }
      else if ( limit_specifier == "x_max" )
      {
         return true;
      }
      else if ( limit_specifier == "y_min" )
      {
         return true;
      }
      else if ( limit_specifier == "y_max" )
      {
         return true;
      }

      return false;
   }

   void process( const coral::cli::ArgumentList& args )
   {
      Adxl345Controller::AccelerationData vector;

      if (args.size() < 1)
      {
         coral::log::error("Expected limit specifier of <x_min,x_max,y_min,y_max>\n");
      }
      else
      {
         const std::string& limit_specifier = args[0];

         if ( valid_limit_specifier(limit_specifier) )
         {
            if ( imu_.read_acceleration_data(vector) )
            {
               if ( limit_specifier == "x_min" )
               {
                  control_.set_limit_x_min(vector.x);
               }
               else if ( limit_specifier == "x_max" )
               {
                  control_.set_limit_x_max(vector.x);
               }
               else if ( limit_specifier == "y_min" )
               {
                  control_.set_limit_y_min(vector.y);
               }
               else if ( limit_specifier == "y_max" )
               {
                  control_.set_limit_y_max(vector.y);
               }
            }
            else
            {
               coral::log::error("ERROR reading from IMU.\n");
            }
         }
         else
         {
            coral::log::error("Invalid limit specifier. Expecting <x_min,x_max,y_min,y_max>.\n");
         }
      }
   }

private:

   Adxl345Controller& imu_;
   GimbalControlThread& control_;
};

class TrackCommand : public InteractiveCommand {
public:

   TrackCommand( GimbalControlThread& control )
      : InteractiveCommand( "track", "Enable/disable tracking" )
      , control_( control )
      {}

   void process( const coral::cli::ArgumentList& args )
   {
      if ( args.size() > 0 )
      {
         const std::string& state = args[0];

         if ( state == "on" )
         {
            if ( !control_.enable_tracking() )
            {
               coral::log::warn("Failed to enable tracking. Verify all settings set.\n");
            }
         }
         else if ( state == "off" )
         {
            control_.disable_tracking();
         }
         else
         {
            coral::log::warn("Expecting arg: <on|off>\n");
         }
      }
      else
      {
         coral::log::warn("Expecting arg: <on|off>\n");
      }
   }

private:

   GimbalControlThread& control_;
};


class SetPointingLimitsCommand : public InteractiveCommand {
public:

   SetPointingLimitsCommand(PanTiltController& pan_tilt)
      : InteractiveCommand( "plim", "Enable/disable tracking" )
      , pan_tilt_(pan_tilt) {}

   void process( const coral::cli::ArgumentList& args )
   {
      const std::string& limit_specifier = args[0];

      if ( limit_specifier == "phi_min" )
      {
         pan_tilt_.set_limit_phi_min( pan_tilt_.get_phi() );
      }
      else if ( limit_specifier == "phi_max" )
      {
         pan_tilt_.set_limit_phi_max( pan_tilt_.get_phi() );
      }
      else if ( limit_specifier == "theta_min" )
      {
         pan_tilt_.set_limit_theta_min( pan_tilt_.get_theta() );
      }
      else if ( limit_specifier == "theta_max" )
      {
         pan_tilt_.set_limit_theta_max( pan_tilt_.get_theta() );
      }
      else
      {
         coral::log::error("Invalid limit specifier. Expecting <phi_min,phi_max,theta_min,theta_max>\n.");
      }
   }

private:

   PanTiltController& pan_tilt_;
};

class LoadLimitsCommand : public InteractiveCommand {
public:

   LoadLimitsCommand(GimbalControlThread& control, PanTiltController& pan_tilt)
      : InteractiveCommand( "load", "Enable/disable tracking" )
      , control_(control)
      , pan_tilt_(pan_tilt) {}

   void process( const coral::cli::ArgumentList& args )
   {
      const std::string file_name = args[0];
      std::ifstream file_stream(file_name, std::ios::in);

      coral::log::status("Reading settings from '%s'...\n", file_name.c_str());

      if ( file_stream.is_open() )
      {
         size_t line_index = 0;

         std::string line;
         while ( std::getline(file_stream, line) )
         {
            std::vector<std::string> tokens = coral::helpers::string_helper::split(line, ' ');

            if ( tokens.size() == 2 )
            {
               if ( tokens[0] == "imu_lim_x_min" )
               {
                  control_.set_limit_x_min(std::stoi(tokens[1]));
               }
               else if ( tokens[0] == "imu_lim_x_max" )
               {
                  control_.set_limit_x_max(std::stoi(tokens[1]));
               }
               else if ( tokens[0] == "imu_lim_y_min" )
               {
                  control_.set_limit_y_min(std::stoi(tokens[1]));
               }
               else if ( tokens[0] == "imu_lim_y_max" )
               {
                  control_.set_limit_y_max(std::stoi(tokens[1]));
               }
               else if ( tokens[0] == "gimbal_lim_phi_min" )
               {
                  pan_tilt_.set_limit_phi_min(deg_to_rad(std::stof(tokens[1])));
               }
               else if ( tokens[0] == "gimbal_lim_phi_max" )
               {
                  pan_tilt_.set_limit_phi_max(deg_to_rad(std::stof(tokens[1])));
               }
               else if ( tokens[0] == "gimbal_lim_theta_min" )
               {
                  pan_tilt_.set_limit_theta_min(deg_to_rad(std::stof(tokens[1])));
               }
               else if ( tokens[0] == "gimbal_lim_theta_max" )
               {
                  pan_tilt_.set_limit_theta_max(deg_to_rad(std::stof(tokens[1])));
               }
               else if ( tokens[0] == "gimbal_speed" )
               {
                  pan_tilt_.set_speed(std::stof(tokens[1]));
               }
               else if ( tokens[0] == "track_method" )
               {
                  if ( tokens[1] == "instant" )
                  {
                     control_.set_track_method(GimbalControlThread::InstantTo);
                  }
                  else
                  {
                     control_.set_track_method(GimbalControlThread::EaseTo);
                  }
               }
               else
               {
                  coral::log::warn("Unrecognized argument at line %d. Skipping.\n", line_index);
               }
            }
            else
            {
               coral::log::warn("Malformed arguments at line %d. Skipping.\n", line_index);
            }

            ++line_index;
         }
      }
      else
      {
         coral::log::error("Error opening file at '%s'\n", file_name.c_str());
      }
   }

private:

   GimbalControlThread& control_;
   PanTiltController&   pan_tilt_;
};

class StoreLimitsCommand : public InteractiveCommand {
public:

   StoreLimitsCommand(GimbalControlThread& control, PanTiltController& pan_tilt)
      : InteractiveCommand( "store", "Enable/disable tracking" )
      , control_(control)
      , pan_tilt_(pan_tilt) {}

   void process( const coral::cli::ArgumentList& args )
   {
      const std::string file_name = args[0];
      std::ofstream file_stream(file_name, std::ios::out);

      if ( file_stream.is_open() )
      {
         file_stream << "imu_lim_x_min " << control_.get_limit_x_min() << std::endl;
         file_stream << "imu_lim_x_max " << control_.get_limit_x_max() << std::endl;

         file_stream << "imu_lim_y_min " << control_.get_limit_y_min() << std::endl;
         file_stream << "imu_lim_y_max " << control_.get_limit_y_max() << std::endl;

         file_stream << "gimbal_lim_phi_min " << static_cast<int16_t>(rad_to_deg(pan_tilt_.get_limit_phi_min())) << std::endl;
         file_stream << "gimbal_lim_phi_max " << static_cast<int16_t>(rad_to_deg(pan_tilt_.get_limit_phi_max())) << std::endl;

         file_stream << "gimbal_lim_theta_min " << static_cast<int16_t>(rad_to_deg(pan_tilt_.get_limit_theta_min())) << std::endl;
         file_stream << "gimbal_lim_theta_max " << static_cast<int16_t>(rad_to_deg(pan_tilt_.get_limit_theta_max())) << std::endl;

         file_stream << "gimbal_speed " << pan_tilt_.get_speed() << std::endl;

         std::string track_method = "instant";
         if (control_.get_track_method() == GimbalControlThread::EaseTo)
         {
            track_method = "ease";
         }
         file_stream << "track_method " << track_method << std::endl;

         file_stream.close();
      }
      else
      {
         coral::log::error("Error opening file at '%s'\n", file_name.c_str());
      }
   }

private:

   GimbalControlThread& control_;
   PanTiltController&   pan_tilt_;
};


int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   I2cInterface& i2c = I2cInterface::instance( "/dev/i2c-1" );

   PwmController     pwm( i2c );
   Adxl345Controller imu( i2c );

   if ( pwm.initialize() )
   {
      if ( imu.initialize() && imu.set_range_setting(Adxl345Controller::kRange2g) )
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

            GimbalControlThread control_thread(pan_tilt, imu);
            control_thread.launch();

            InteractiveCommandRouter router;

            router.add( std::make_shared<PanCommand>(pan_tilt) );
            router.add( std::make_shared<TiltCommand>(pan_tilt) );
            router.add( std::make_shared<PointCommand>(pan_tilt) );
            router.add( std::make_shared<GetPointCommand>(pan_tilt) );
            router.add( std::make_shared<SetSpeedCommand>(pan_tilt) );
            router.add( std::make_shared<ReadImuCommand>(imu) );
            router.add( std::make_shared<CalibrateImuCommand>(imu, control_thread) );
            router.add( std::make_shared<TrackCommand>(control_thread) );
            router.add( std::make_shared<SetPointingLimitsCommand>(pan_tilt) );
            router.add( std::make_shared<LoadLimitsCommand>(control_thread, pan_tilt) );
            router.add( std::make_shared<StoreLimitsCommand>(control_thread, pan_tilt) );
            
            router.run();

            control_thread.cancel(true);
         }
         else
         {
            log::error("Initialization failed.\n");
         }
      }
      else
      {
         log::error("Failed to initialize IMU.\n");
      }
   }
   else
   {
      log::error("Failed to initialize PWM controller.\n");
   }

   log::flush();

   return 0;
}
