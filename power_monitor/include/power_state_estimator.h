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

#ifndef POWER_MONITOR_POWER_STATE_ESTIMATOR_H
#define POWER_MONITOR_POWER_STATE_ESTIMATOR_H

#include <stdlib.h>

#include <string>
#include <vector>

#include "rclcpp/duration.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

#include "observation.h"

namespace power_monitor {

struct PowerStateEstimate
{
    rclcpp::Duration time_remaining{0, 0};
    unsigned int  relative_capacity;
};

/**
 * PowerStateEstimator can takes observable power inputs (PowerObservation) and estimates the hidden power state (PowerStateEstimate).
 */
class PowerStateEstimator
{
public:
    enum Type { FuelGauge, Advanced };

    PowerStateEstimator();

    virtual std::string               getMethodName() const = 0;
    virtual PowerStateEstimator::Type getMethodType() const = 0;
    virtual PowerStateEstimate        estimate(const rclcpp::Time& t) = 0;

    virtual void recordObservation(const PowerObservation& obs);

    virtual bool canEstimate(const rclcpp::Time& t) const;

protected:
    PowerObservation obs_;

    // ROS 1: ros::Time::now(); ROS 2: a ROS clock (system time without sim time, like the node's).
    rclcpp::Clock clock_{RCL_ROS_TIME};
};

/** A simple power state estimator which relies on the values reported by the batteries themselves
  * for the minimum & maximum time remaining and the minimum remaining capacity.
  */
class FuelGaugePowerStateEstimator : public PowerStateEstimator
{
public:
    std::string               getMethodName() const;
    PowerStateEstimator::Type getMethodType() const;
    PowerStateEstimate        estimate(const rclcpp::Time& t);
};

/** A more advanced power state estimator which is informed by the history of the batteries.
  */
class AdvancedPowerStateEstimator : public PowerStateEstimator
{
public:
    struct LogRecord
    {
        uint32_t     sec;
        int8_t       master_state;
        int          charging;
        float        total_power;
        float        min_voltage;
        unsigned int min_relative_state_of_charge;
        float        total_remaining_capacity;
    };

    static const std::string DEFAULT_LOG_FILE;

    // ROS 1 read ~advanced_log_file itself; ROS 2: the node passes it (default DEFAULT_LOG_FILE).
    explicit AdvancedPowerStateEstimator(const std::string& log_filename);

    std::string               getMethodName() const;
    PowerStateEstimator::Type getMethodType() const;
    PowerStateEstimate        estimate(const rclcpp::Time& t);

    virtual void recordObservation(const PowerObservation& obs);

private:
    static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters=",");

    bool readObservations(std::vector<LogRecord>& log);
    bool saveObservation(const PowerObservation& obs) const;
    bool hasEverDischarged() const;

    std::vector<LogRecord> log_;

    std::string log_filename_;
};

}

#endif /* POWER_MONITOR_POWER_STATE_ESTIMATOR_H */
