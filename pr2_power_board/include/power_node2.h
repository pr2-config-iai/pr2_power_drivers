
#pragma once

#include <time.h>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "pr2_msgs/msg/battery_server2.hpp"
#include "pr2_msgs/msg/power_board_state.hpp"
#include "pr2_power_board/srv/power_board_command.hpp"
#include "pr2_power_board/srv/power_board_command2.hpp"

class Interface 
{
  public:

    int recv_sock;
    int send_sock;
    Interface();
    ~Interface() {Close();}
    void Close();
    int Init( const std::string &address_str);
    int InitReceive(const std::string &address_str);
    void AddToReadSet(fd_set &set, int &max_sock) const;
    bool IsReadSet(fd_set set) const;
};


class Device 
{
  public:
    rclcpp::Time message_time;
    
    const TransitionMessage &getTransitionMessage()
    {
      return tmsg;
    }
    void setTransitionMessage(const TransitionMessage &newtmsg);
    
    const PowerMessage &getPowerMessage()
    {
      return pmsg;
    }
    void setPowerMessage(const PowerMessage &newpmsg);
    
    Device();
    ~Device() { };
  private:
    bool tmsgset;
    TransitionMessage tmsg;
    bool pmsgset;
    PowerMessage pmsg;  //last power message recived from device
};


class PowerBoard
{
  public:
    PowerBoard( const rclcpp::Node::SharedPtr &node, const std::string &address_str );
    void commandCallback( const std::shared_ptr<pr2_power_board::srv::PowerBoardCommand::Request> req_,
                          std::shared_ptr<pr2_power_board::srv::PowerBoardCommand::Response> res_);
    void commandCallback2( const std::shared_ptr<pr2_power_board::srv::PowerBoardCommand2::Request> req_,
                          std::shared_ptr<pr2_power_board::srv::PowerBoardCommand2::Response> res_);
    void init();
    void collectMessages();
    void sendMessages();
    int collect_messages();
    int process_message(const PowerMessage *msg, int len);
    int process_transition_message(const TransitionMessage *msg, int len);
    const char* master_state_to_str(char state);
    const char* cb_state_to_str(char state);
    int list_devices(void);
    int send_command(int circuit_breaker, const std::string &command, unsigned flags);
    int requestMessage(const unsigned int message);

    void checkFanSpeed(); // Check battery temperatures and send command for fan speed

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<pr2_power_board::srv::PowerBoardCommand>::SharedPtr service;
    rclcpp::Service<pr2_power_board::srv::PowerBoardCommand2>::SharedPtr service2;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diags_pub;
    rclcpp::Publisher<pr2_msgs::msg::PowerBoardState>::SharedPtr state_pub;
    rclcpp::Subscription<pr2_msgs::msg::BatteryServer2>::SharedPtr battery_sub_;

    std::map<int, float> battery_temps_;
    bool fan_high_;

    int getFanDuty(); // Duty cycle to send to fan. 0 if no fan command
  
    void batteryCB(const pr2_msgs::msg::BatteryServer2::ConstSharedPtr &msgPtr);

    std::mutex library_lock_;
    uint64_t ip_address;

};
