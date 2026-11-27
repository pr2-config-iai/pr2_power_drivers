#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <boost/program_options.hpp>

#include "ocean.h"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "pr2_msgs/msg/battery_server.hpp" //This is here to send a copy of the previous message

using namespace std;
using namespace willowgarage::ocean;
namespace po = boost::program_options;
using diagnostic_msgs::msg::DiagnosticStatus;

static const float BATTERY_TEMP_WARN = 50.0;

float toFloat(const int &value)
{
  int tmp = value;
  if(tmp & 0x8000)
    tmp = tmp - 65536;
  float result = tmp / 1000.0;
  return result;
}

float tempToCelcius(const int &iKelvin)
{
  float fKelvin = (float) (iKelvin * 0.1);
  return (fKelvin - (float) 273.15);
}

// ROS 1 gave every server its own node handle and publishers; ROS 2: the node and the publishers
// are shared (rclcpp publishers are thread-safe), and lag_timeout/stale_timeout are read once in main.
struct Shared
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub;
  rclcpp::Publisher<pr2_msgs::msg::BatteryServer2>::SharedPtr bs2;
  rclcpp::Publisher<pr2_msgs::msg::BatteryServer>::SharedPtr bs;
  int lag_timeout, stale_timeout;
};

class server
{
  public:

    server( const Shared &shared, const int &majorID, const std::string &dev, const int debug_level = 0 ) :
      shared(shared), majorID(majorID), debug_level(debug_level), serial_device("/dev/ttyUSB0"), stopRequest(false),
      has_warned_temp_alarm_(false), has_warned_no_good_(false)
    {
      std::stringstream ss;

      if(dev.empty())
      {
        string tmp_device;
        ss.str("");
        ss << "/dev/ttyUSB" << majorID; //create a default device based on majorID
        serial_device = ss.str();

        ss.str("");
        ss << "port" << majorID;
        // ROS 1: getParam (unset -> default); ROS 2: an empty string means unset.
        tmp_device = shared.node->declare_parameter<std::string>(ss.str(), "");
        if(!tmp_device.empty())
        {
          RCLCPP_INFO(logger(), "Using %s from getParam.\n", ss.str().c_str());
          serial_device = tmp_device;
        }
        else
          RCLCPP_INFO(logger(), "Defaulting to: %s", serial_device.c_str());

      }
      else
      {
        RCLCPP_INFO(logger(), "Overriding device with argument: %s", dev.c_str() );
        serial_device = dev;
      }

      lag_timeout_ = shared.lag_timeout;
      stale_timeout_ = shared.stale_timeout;
    }

    void start()
    {
      myThread = std::make_shared<std::thread>(&server::run, this);
    }


    void stop()
    {
      stopRequest = true;
      myThread->join();
    }

  private:

    rclcpp::Logger logger() const { return shared.node->get_logger(); }

    Shared shared;
    int majorID;
    int debug_level;
    std::string serial_device;
    volatile bool stopRequest;
    std::shared_ptr<std::thread> myThread;
    int lag_timeout_, stale_timeout_;
    bool has_warned_temp_alarm_, has_warned_no_good_;

    void run()
    {
      std::stringstream ss;

      //
      //  Need to make the queue size big enough that each thread can publish without
      //  concern that one message it quickly replaced by another threads message.
      //
      // ROS 1 advertised /diagnostics, battery/server2 and battery/server here (queue 10), once per
      // server; ROS 2: the shared publishers (main).
      auto &pub = shared.pub;
      auto &bs2 = shared.bs2;
      auto &bs  = shared.bs;

      rclcpp::Rate rate(100);   //set the rate we scan the device for input
      diagnostic_msgs::msg::DiagnosticArray msg_out;
      diagnostic_updater::DiagnosticStatusWrapper stat;
      rclcpp::Time lastTime(0, 0, RCL_ROS_TIME), currentTime(0, 0, RCL_ROS_TIME), startTime(0, 0, RCL_ROS_TIME);
      const rclcpp::Duration MESSAGE_TIME(2,0);    //the message output rate
      ocean os( majorID, debug_level);
      os.initialize(serial_device.c_str());
      //os.read_file(serial_device.c_str());

      pr2_msgs::msg::BatteryServer oldserver;
      oldserver.battery.resize(4);

      lastTime = shared.node->now();
      startTime = shared.node->now();

      while(rclcpp::ok() && (stopRequest == false))
      {
        rate.sleep();
        //ros::spinOnce();
        currentTime = shared.node->now();

        if((os.run() > 0) && ((currentTime - lastTime) > MESSAGE_TIME))
        {

          // First publish our internal data
          os.server.header.stamp = shared.node->now();
          bs2->publish(os.server);

          oldserver.id = os.server.id;
          oldserver.last_time_system = os.server.last_system_update.sec;
          oldserver.time_left = rclcpp::Duration(os.server.time_left).seconds()/60;
          oldserver.average_charge = os.server.average_charge;
          oldserver.message = os.server.message;
          oldserver.last_time_controller = os.server.last_controller_update.sec;
          oldserver.present = os.server.battery[0].present * 1 + os.server.battery[1].present * 2 + os.server.battery[2].present * 4 + os.server.battery[3].present * 8;
          oldserver.charging = os.server.battery[0].charging * 1 + os.server.battery[1].charging * 2 + os.server.battery[2].charging * 4 + os.server.battery[3].charging * 8;
          oldserver.discharging = os.server.battery[0].discharging * 1 + os.server.battery[1].discharging * 2 + os.server.battery[2].discharging * 4 + os.server.battery[3].discharging * 8;
          //oldserver.reserved = os.server.battery[0].reserved * 1 + os.server.battery[1].reserved * 2 + os.server.battery[2].reserved * 4 + os.server.battery[3].reserved * 8;
          oldserver.power_present = os.server.battery[0].power_present * 1 + os.server.battery[1].power_present * 2 + os.server.battery[2].power_present * 4 + os.server.battery[3].power_present * 8;
          oldserver.power_ng = os.server.battery[0].power_no_good * 1 + os.server.battery[1].power_no_good * 2 + os.server.battery[2].power_no_good * 4 + os.server.battery[3].power_no_good * 8;
          oldserver.inhibited = os.server.battery[0].inhibited * 1 + os.server.battery[1].inhibited * 2 + os.server.battery[2].inhibited * 4 + os.server.battery[3].inhibited * 8;

          for(int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            oldserver.battery[xx].last_time_battery = os.server.battery[xx].last_battery_update.sec;
            for(unsigned int yy = 0; yy < os.regListLength; ++yy)
            {
              oldserver.battery[xx].bat_reg[yy] = os.server.battery[xx].battery_register[yy];
              oldserver.battery[xx].bat_reg_flag[yy] = os.server.battery[xx].battery_update_flag[yy];
              oldserver.battery[xx].bat_reg_time[yy] = os.server.battery[xx].battery_register_update[yy].sec;
            }
          }

          oldserver.header.stamp = shared.node->now();
          bs->publish(oldserver);

          lastTime = currentTime;

          stat.values.clear();

          ss.str("");
          ss << "IBPS " << majorID;
          stat.name = ss.str();
          stat.level = DiagnosticStatus::OK;
          stat.message = "OK";
          
          stat.add("Time Remaining (min)", (rclcpp::Duration(os.server.time_left).seconds()/60));
          stat.add("Average charge (percent)", os.server.average_charge );
          rclcpp::Duration elapsed = currentTime - rclcpp::Time(os.server.last_system_update, RCL_ROS_TIME);
          stat.add("Time since update (s)", elapsed.seconds());

          msg_out.header.stamp = shared.node->now();
          msg_out.status.push_back(stat);

          for(int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            stat.values.clear();
            
            ss.str("");
            ss << "Smart Battery " << majorID << "." << xx;
            stat.name = ss.str();
            stat.level = DiagnosticStatus::OK;
            stat.message = "OK";

            if(!os.server.battery[xx].present)
            {
              stat.add("Battery Present", "False");
              stat.level = DiagnosticStatus::ERROR;
              stat.message = "Not present";
            }
            else
            {
              stat.add("Battery Present", "True");
              stat.add("Charging", (os.server.battery[xx].charging) ? "True":"False");
              stat.add("Discharging", (os.server.battery[xx].discharging) ? "True":"False");
              stat.add("Power Present", (os.server.battery[xx].power_present) ? "True":"False");
              stat.add("No Good", (os.server.battery[xx].power_no_good) ? "True":"False");
              stat.add("Charge Inhibited", (os.server.battery[xx].inhibited) ? "True":"False");

              int16_t voltage = -1;
              int16_t design_voltage = -1;
              int16_t relative_charge = -1;
              int16_t absolute_charge = -1;
              
              for(unsigned int yy = 0; yy < os.regListLength; ++yy)
              {
                unsigned addr = os.regList[yy].address;
                if(os.server.battery[xx].battery_update_flag[addr])
                {
                  ss.str("");
                  if(os.regList[yy].unit != "")
                    ss << os.regList[yy].name << " (" << os.regList[yy].unit << ")";
                  else
                    ss << os.regList[yy].name;
                  
                  switch(addr) {
                     case 0x1b:  //Address of manufactureDate
                        {
                           std::stringstream date;
                           date.str("");

                           unsigned int day = os.server.battery[xx].battery_register[addr] & 0x1F;
                           unsigned int month = (os.server.battery[xx].battery_register[addr] >> 5) & 0xF;
                           unsigned int year = (os.server.battery[xx].battery_register[addr] >> 9) + 1980;
                           date << month << "/" << day << "/" << year;

                           stat.add("Manufacture Date (MDY)", date.str());
                        }
                        break;
                     case 0x8:  //Address of Temperature
                        {
                           float celsius = tempToCelcius(os.server.battery[xx].battery_register[addr]);
                           if(celsius > BATTERY_TEMP_WARN)
                           {
                              ostringstream warn;
                              warn << "High Temperature Warning > " << BATTERY_TEMP_WARN << "C";
                              stat.mergeSummary(DiagnosticStatus::WARN, warn.str());
                           }

                           stat.add("Temperature (C)", celsius);
                        }
                        break;
                     case 0x1c: // Serial Number
                        {
                           stat.add("Serial Number", os.server.battery[xx].battery_register[addr]);

                           std::stringstream hw_id;
                           hw_id << os.server.battery[xx].battery_register[addr];

                           stat.hardware_id = hw_id.str();
                        }
                        break;
                     case 0x09: // Voltage
                        voltage = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     case 0x19: // Design Voltage
                        design_voltage = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     case 0x0d: // Relative state of charge
                        relative_charge = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     case 0x0e: // Absolute state of charge
                        absolute_charge = os.server.battery[xx].battery_register[addr];
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                     default:
                        stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                        break;
                  }
                }
              }

              const rclcpp::Time last_battery_update(os.server.battery[xx].last_battery_update, RCL_ROS_TIME);
              elapsed = currentTime - last_battery_update;
              if (last_battery_update >= rclcpp::Time(1, 0, RCL_ROS_TIME))
                stat.add("Time since update (s)", elapsed.seconds());
              else
                stat.add("Time since update (s)", "N/A");
              
              if ( absolute_charge >= 0 && relative_charge >= 0 ) {
                 if( absolute_charge < relative_charge/2 ) {
                    stat.mergeSummary(DiagnosticStatus::WARN, "Battery capacity low, please replace.");
                 }
              }

              if ( voltage >= 0 && design_voltage >= 0 ) {
                 if( voltage < design_voltage/2 ) {
                    stat.mergeSummary(DiagnosticStatus::WARN, "Battery voltage too low, please replace.");
                 }
              }

              // Mark batteries as stale if they don't update
              // Give them "grace period" on startup to update before we report error
              bool updateGracePeriod = (currentTime - startTime).seconds() < stale_timeout_;
              if (last_battery_update <= rclcpp::Time(1, 0, RCL_ROS_TIME) && updateGracePeriod)
                stat.mergeSummary(DiagnosticStatus::WARN, "Waiting for battery update");
              else if (stale_timeout_ > 0 && elapsed.seconds() > stale_timeout_)
                stat.mergeSummary(DiagnosticStatus::ERROR, "No updates");
              else if (lag_timeout_ > 0 && elapsed.seconds() > lag_timeout_)
                stat.mergeSummary(DiagnosticStatus::WARN, "Stale updates");


              // Warn for over temp alarm
              // If power present and not charging, not full, and temp >= 46C
              // 0x0d is "Relative State of Charge"
              if (os.server.battery[xx].power_present && !os.server.battery[xx].charging 
                  && os.server.battery[xx].battery_register[0x0d] < 90
                  && tempToCelcius(os.server.battery[xx].battery_register[0x8]) > 46.0)
                {
                  stat.mergeSummary(DiagnosticStatus::WARN, "Charge Inhibited, High Temperature");
                  if (!has_warned_temp_alarm_)
                    {
                      RCLCPP_WARN(logger(), "Over temperature alarm found on battery %d. Battery will not charge.", xx);
                      has_warned_temp_alarm_ = true;
                    }
                }

              // Check for battery status code, not sure if this works.
              if (os.server.battery[xx].battery_register[0x16] & 0x1000)
                  stat.mergeSummary(DiagnosticStatus::WARN, "Over Temp Alarm");

              // Report a console warning if battery is "No Good"
              // This may be a problem with the battery, but we're not sure.
              if (os.server.battery[xx].power_no_good && !has_warned_no_good_)
                {
                  RCLCPP_WARN(logger(), "Battery %d reports \"No Good\".", xx);
                  has_warned_no_good_ = true;
                }
                          
              msg_out.status.push_back(stat);
            }
          }

          pub->publish(msg_out);
          msg_out.status.clear();

        }
      }
    }

};

int main(int argc, char** argv)
{
  string tmp_device;
  int debug_level;
  int max_ports;

  rclcpp::init(argc, argv);
  const std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<const char *> args;
  for (const auto &a : non_ros_args)
    args.push_back(a.c_str());

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("debug", po::value<int>(&debug_level)->default_value(0), "debug level")
    ("count", po::value<int>(&max_ports)->default_value(4), "number of ports to monitor")
    ("dev", po::value<string>(&tmp_device), "serial device to open");

  po::variables_map vm;
  po::store(po::parse_command_line( static_cast<int>(args.size()), args.data(), desc), vm);
  po::notify(vm);

  if( vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("ocean_server");

  //majorID = serial_device.at(serial_device.length() - 1) - '0';

  // ROS 1 set the log4cxx logger to Info unless someone had set its level; Info is ROS 2's default.

  // ROS 1: the parameters override the command line when they are set.
  max_ports = node->declare_parameter<int>( "number_of_ports", max_ports );
  RCLCPP_INFO(node->get_logger(), "number_of_ports=%d", max_ports);
  debug_level = node->declare_parameter<int>( "debug_level", debug_level );
  RCLCPP_DEBUG(node->get_logger(), "debug_level=%d", debug_level);

  Shared shared;
  shared.node = node;
  shared.lag_timeout = node->declare_parameter<int>("lag_timeout", 60);
  shared.stale_timeout = node->declare_parameter<int>("stale_timeout", 120);
  //
  //  Need to make the queue size big enough that each thread can publish without
  //  concern that one message it quickly replaced by another threads message.
  //
  shared.pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  shared.bs2 = node->create_publisher<pr2_msgs::msg::BatteryServer2>("battery/server2", 10);
  shared.bs  = node->create_publisher<pr2_msgs::msg::BatteryServer>("battery/server", 10);

  vector<std::unique_ptr<server>> server_list;

  for(int xx = 0; xx < max_ports; ++xx)
    server_list.push_back(std::make_unique<server>(shared, xx, tmp_device, debug_level));

  for(int xx = 0; xx < max_ports; ++xx)
    server_list[xx]->start();

  rclcpp::spin(node); //wait for ros to shut us down


  for(int xx = 0; xx < max_ports; ++xx)
    server_list[xx]->stop();

  rclcpp::shutdown();
  return 0;
}
