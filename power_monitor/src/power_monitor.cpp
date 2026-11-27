/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "power_monitor.h"

#include <chrono>

using namespace std;
using namespace power_monitor;

PowerMonitor::PowerMonitor() : rclcpp::Node("power_monitor"),
                               master_state_(-1),
                               battery_update_timeout_(120)
{
    static const string battery_server_topic = "battery/server2";
    static const string power_board_node     = "power_board";
    string estimator_type_str                = "advanced";
    double freq                              = 0.1;

    estimator_type_str      = declare_parameter<string>("estimation_method",      estimator_type_str);
    freq                    = declare_parameter<double>("frequency",              freq);
    battery_update_timeout_ = declare_parameter<double>("battery_update_timeout", battery_update_timeout_);
    // ROS 1: read by the AdvancedPowerStateEstimator from ~advanced_log_file.
    const string advanced_log_file = declare_parameter<string>("advanced_log_file", AdvancedPowerStateEstimator::DEFAULT_LOG_FILE);

    rclcpp::sleep_for(std::chrono::seconds(2));

    // Register the estimators
    addEstimator(new FuelGaugePowerStateEstimator());
    addEstimator(new AdvancedPowerStateEstimator(advanced_log_file));

    // Setup the dynamic_reconfigure callback
    // ROS 2: the parameter power_state_estimator_ (cfg/PowerMonitor.cfg: 0 FuelGauge, 1 Advanced, default 0).
    // Like dynamic_reconfigure's setCallback, the initial value is applied once right away.
    rcl_interfaces::msg::ParameterDescriptor estimator_desc;
    estimator_desc.description = "Int enum: 0 FuelGauge (the fuel gauge estimator), 1 Advanced (the advanced estimator)";
    estimator_desc.integer_range.resize(1);
    estimator_desc.integer_range[0].from_value = 0;
    estimator_desc.integer_range[0].to_value   = 1;
    estimator_desc.integer_range[0].step       = 1;
    const int64_t initial_estimator = declare_parameter<int64_t>("power_state_estimator_", 0, estimator_desc);
    setActiveEstimator((PowerStateEstimator::Type) initial_estimator);
    config_server_ = add_on_set_parameters_callback(std::bind(&PowerMonitor::configCallback, this, std::placeholders::_1));

    // Set the active estimation method
    if (estimator_types_.size() == 0)
    {
        RCLCPP_FATAL(get_logger(), "No power state estimators defined. Shutting down");
        exit(1);
    }
    map<string, PowerStateEstimator::Type>::const_iterator i = estimator_types_.find(estimator_type_str);
    if (i == estimator_types_.end())
    {
        // Requested estimator is unknown. Default to first estimator
        string first_estimator_type_str = estimator_types_.begin()->first;
        RCLCPP_ERROR(get_logger(), "Unknown power state estimator type: %s. Defaulting to %s", estimator_type_str.c_str(), first_estimator_type_str.c_str());
        setActiveEstimator(estimator_types_.begin()->second);
    }
    else
        setActiveEstimator(i->second);

    // ROS 1: latched (queue 5): a late subscriber gets the last message -> transient_local, depth 1.
    power_state_pub_       = create_publisher<pr2_msgs::msg::PowerState>("power_state", rclcpp::QoS(1).transient_local());
    // ROS 1: a ROS-time timer -> a timer on the node's clock.
    power_state_pub_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / freq),
                                                  std::bind(&PowerMonitor::onPublishTimer, this));
    battery_server_sub_    = create_subscription<pr2_msgs::msg::BatteryServer2>(battery_server_topic, 10,
                                 std::bind(&PowerMonitor::batteryServerUpdate, this, std::placeholders::_1));
    // power_board/state is latched (transient_local); ROS 1 subscribers got the latched message too.
    power_node_sub_        = create_subscription<pr2_msgs::msg::PowerBoardState>(power_board_node + "/state", rclcpp::QoS(10).transient_local(),
                                 std::bind(&PowerMonitor::powerNodeUpdate, this, std::placeholders::_1));
}

void PowerMonitor::addEstimator(PowerStateEstimator* est)
{
    estimator_types_[est->getMethodName()] = est->getMethodType();
    estimators_[est->getMethodType()] = std::shared_ptr<PowerStateEstimator>(est);
}

rcl_interfaces::msg::SetParametersResult PowerMonitor::configCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    for (const rclcpp::Parameter& p : parameters)
        if (p.get_name() == "power_state_estimator_")
            setActiveEstimator((PowerStateEstimator::Type) p.as_int());

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

bool PowerMonitor::setActiveEstimator(PowerStateEstimator::Type estimator_type)
{
    map<PowerStateEstimator::Type, std::shared_ptr<PowerStateEstimator> >::const_iterator i = estimators_.find(estimator_type);
    if (i == estimators_.end())
        return false;
    if (active_estimator_ == i->second)
        return true;

    if (active_estimator_ == std::shared_ptr<PowerStateEstimator>())
        RCLCPP_INFO(get_logger(), "Power state estimator set to %s", i->second->getMethodName().c_str());
    else
        RCLCPP_INFO(get_logger(), "Power state estimator changed from %s to %s", active_estimator_->getMethodName().c_str(), i->second->getMethodName().c_str());

    active_estimator_ = i->second;

    return true;
}

void PowerMonitor::batteryServerUpdate(const pr2_msgs::msg::BatteryServer2::ConstSharedPtr& battery_server)
{
    std::lock_guard<std::mutex> lock(update_mutex_);

    RCLCPP_DEBUG(get_logger(), "Received battery message: voltage=%.2f", battery_server->battery[0].battery_register[0x9] / 1000.0f);

    battery_servers_[battery_server->id] = battery_server;
}

void PowerMonitor::powerNodeUpdate(const pr2_msgs::msg::PowerBoardState::ConstSharedPtr& power_board_state)
{
    {
        // ROS 1 still held this lock while publishing below; extractObservation() locks it again, so
        // ROS 1 deadlocked on MASTER_SHUTDOWN (boost::mutex is not recursive). ROS 2 releases it first.
        std::lock_guard<std::mutex> lock(update_mutex_);

        RCLCPP_DEBUG(get_logger(), "Received power board state message: %s", masterStateToString(power_board_state->master_state).c_str());

        master_state_ = power_board_state->master_state;
    }

    // Publish the power state immediately if we're shutting down. Want to ensure we record these data points.
    if (power_board_state->master_state == pr2_msgs::msg::PowerBoardState::MASTER_SHUTDOWN)
    {
      RCLCPP_WARN(get_logger(), "Power board reports imminant shutdown");
      publishPowerState();
    }
}

string PowerMonitor::masterStateToString(int8_t master_state) const
{
    switch (master_state)
    {
        case pr2_msgs::msg::PowerBoardState::MASTER_NOPOWER:  return "No Power";
        case pr2_msgs::msg::PowerBoardState::MASTER_STANDBY:  return "Standby";
        case pr2_msgs::msg::PowerBoardState::MASTER_ON:       return "On";
        case pr2_msgs::msg::PowerBoardState::MASTER_OFF:      return "Off";
        case pr2_msgs::msg::PowerBoardState::MASTER_SHUTDOWN: return "Shutdown";
        default:                                              return "Unknown";
    }
}

PowerObservation PowerMonitor::extractObservation()
{
    std::lock_guard<std::mutex> lock(update_mutex_);

    vector<BatteryObservation> batteries;
    for (map<int, pr2_msgs::msg::BatteryServer2::ConstSharedPtr>::iterator i = battery_servers_.begin(); i != battery_servers_.end(); i++)
    {
        const pr2_msgs::msg::BatteryServer2* bs = i->second.get();

        rclcpp::Time stamp(bs->header.stamp, RCL_ROS_TIME);

        for (unsigned int j = 0; j < bs->battery.size(); j++)
        {
            const pr2_msgs::msg::BatteryState2& b = bs->battery[j];

            bool         ac_present = b.power_present;
            float        voltage    = b.battery_register[0x9] / 1000.0f;
            float        current    = b.battery_register[0xA] / 1000.0f;
            unsigned int rsc        = b.battery_register[0x0D];
            float        rem_cap    = b.battery_register[0x0F] / 1000.0f;
            unsigned int tte_min    = b.battery_register[0x12];
            unsigned int ttf_min    = b.battery_register[0x13];

            rclcpp::Duration tte(-1, 0);
            if (tte_min > 0)
                tte = rclcpp::Duration(tte_min * 60, 0);
            else
                tte = rclcpp::Duration(-1, 0);

            rclcpp::Duration ttf(-1, 0);
            if (ttf_min > 0)
                ttf = rclcpp::Duration(ttf_min * 60, 0);
            else
                ttf = rclcpp::Duration(-1, 0);

            if (voltage == 0.0)
                continue;

            batteries.push_back(BatteryObservation(stamp, ac_present, voltage, current, rsc, rem_cap, tte, ttf));

            RCLCPP_DEBUG(get_logger(), "Battery %d.%d: %6.2f V %6.2f A %6.2f W (soc: %d, cap: %6.2f, tte: %dm, ttf: %dm)", bs->id, j + 1, voltage, current, current * voltage, rsc, rem_cap, tte_min, ttf_min);
        }
    }

    return PowerObservation(now(), master_state_, batteries);
}

rclcpp::Time PowerMonitor::getLastBatteryUpdate() const
{
  rclcpp::Time rv(0, 0, RCL_ROS_TIME);
  for (map<int, pr2_msgs::msg::BatteryServer2::ConstSharedPtr>::const_iterator i = battery_servers_.begin(); i != battery_servers_.end(); i++)
  {
    const pr2_msgs::msg::BatteryServer2* bs = i->second.get();
    const rclcpp::Time stamp(bs->header.stamp, RCL_ROS_TIME);
    if (stamp > rv)
      rv = stamp;
  }

  return rv;
}

void PowerMonitor::onPublishTimer()
{
  // Don't publish power data if we haven't received battery data in a timeout, #4851
  if (battery_update_timeout_ > 0 && ((now() - getLastBatteryUpdate()).seconds() > battery_update_timeout_))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "Power monitor not publishing estimate, batteries have not recently updated. Check diagnostics.");
    return;
  }

  publishPowerState();
}

void PowerMonitor::publishPowerState()
{
    std::lock_guard<std::mutex> lock(publish_mutex_);

    // Extract info from the battery server
    PowerObservation obs = extractObservation();
    if (obs.getBatteries().size() == 0)
    {
        RCLCPP_DEBUG(get_logger(), "Nothing observed");
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Power: %6.1f W. Min voltage: %6.2f V", obs.getTotalPower(), obs.getMinVoltage());

    // Give every estimator the chance to record this observation
    for (map<PowerStateEstimator::Type, std::shared_ptr<PowerStateEstimator> >::const_iterator i = estimators_.begin(); i != estimators_.end(); i++)
        i->second.get()->recordObservation(obs);

    // Use the active estimator to estimate the time and capacity remaining
    rclcpp::Time t = now();
    if (active_estimator_->canEstimate(t))
    {
        PowerStateEstimate estimate = active_estimator_->estimate(t);
        RCLCPP_DEBUG(get_logger(), "Remaining: %.0f min (%d%%)", estimate.time_remaining.seconds() / 60, estimate.relative_capacity);

        // Publish the power state estimate
        pr2_msgs::msg::PowerState ps;
        ps.header.stamp      = now();
        ps.ac_present        = obs.getAcCount();
        ps.power_consumption = obs.getTotalPower();
        ps.prediction_method = active_estimator_->getMethodName();
        ps.relative_capacity = (int8_t) estimate.relative_capacity;
        ps.time_remaining    = estimate.time_remaining;
        power_state_pub_->publish(ps);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerMonitor>());
    rclcpp::shutdown();
    return 0;
}
