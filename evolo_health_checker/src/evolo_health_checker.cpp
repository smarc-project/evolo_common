/**
 * @author Niklas Rolleberg
 * @author_email nrol@kth.se
 */
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "smarc_msgs/msg/geofence_status_stamped.hpp"
#include "smarc_msgs/msg/topics.hpp"
#include "evolo_msgs/msg/topics.hpp"
#include "evolo_msgs/msg/captain_state.hpp"


#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"

using namespace std::chrono_literals;


class ErrorBucket {
private:
  float volume = 0; //0 = empty 1 = full
  float errorThreshold = 0.75;
  float drainRate = 0.05;
  float fillRate_bool = 0.24;
public:
  ErrorBucket(float _drainRate = 0.05, float _errorThreshhold = 0.75, float _fillRate_bool = 0.24) { //goes from full to empty in 20 updates
    drainRate = _drainRate;
    errorThreshold = _errorThreshhold;
    fillRate_bool = _fillRate_bool;
  };

  //Fill bucket with custom amout
  bool update(float fill = 0) {
    volume -= drainRate;
    if(volume < 0) volume = 0;
    volume += fill;
    if(volume > 1) volume = 1;
    return volume < errorThreshold;
  }

  //Fill bucket with standard amount
  bool update(bool OK = true) {
    volume -= drainRate;
    if(volume < 0) volume = 0;
    if(!OK) volume += fillRate_bool;
    if(volume > 1) volume = 1;
    return volume < errorThreshold;
  }

  bool getStatus() {
    return volume < errorThreshold;
  }

  float getVolume() {
    return volume;
  }

};

class HealthChecker : public rclcpp::Node {
 public:
  HealthChecker() : Node("evolo_health_checker") {

    //Subscribers
    // odom;
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      smarc_msgs::msg::Topics::ODOM_TOPIC, 10, std::bind(&HealthChecker::odomCallback, this, std::placeholders::_1)
    );

    // SBG nav sub
    _sbg_nav_sub = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
      "sbg/ekf_nav", 10, std::bind(&HealthChecker::sbgNavCallback, this, std::placeholders::_1)
    );

    // Captain status callback
    _captain_state_sub = this->create_subscription<evolo_msgs::msg::CaptainState>(
      "/evolo/captain/state", 10, std::bind(&HealthChecker::captainStateCallback, this, std::placeholders::_1)
    );

    //Geofence sub
    _geofence_sub = this->create_subscription<smarc_msgs::msg::GeofenceStatusStamped>(
      smarc_msgs::msg::Topics::GEOFENCE_STATUS_TOPIC, 10, 
      std::bind(&HealthChecker::geofenceCallback, this, std::placeholders::_1));


    // Occupancy grid
    _grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map_in_topic", 10,
      std::bind(&HealthChecker::occupancyGridCallback, this, std::placeholders::_1)
      );

    // tf listener for checking if TF is OK
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer for checking
    healthcheck_timer = this->create_wall_timer(
        1000ms, std::bind(&HealthChecker::timer_callback, this));

    //Health output pub
    _health_pub = this->create_publisher<std_msgs::msg::Int8>("/evolo/smarc/vehicle_health", 10);
  }

 public:
  
 //Settings?

 private:

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr _sbg_nav_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _grid_sub;
  rclcpp::Subscription<evolo_msgs::msg::CaptainState>::SharedPtr _captain_state_sub;
  rclcpp::Subscription<smarc_msgs::msg::GeofenceStatusStamped>::SharedPtr _geofence_sub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp::TimerBase::SharedPtr healthcheck_timer;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _health_pub;


  // messages
  nav_msgs::msg::Odometry odom_message;
  bool odom_received = false; // True if we have received at least one odom message
  bool odom_updated = false; // True if we have received an upate since last check

  sbg_driver::msg::SbgEkfNav ekfNav_message;
  bool ekfNav_received = false;
  bool ekfNav_updated = false;

  evolo_msgs::msg::CaptainState captain_state_msg;
  bool captain_received = false;
  bool captain_updated = false;

  nav_msgs::msg::OccupancyGrid map_message;
  bool map_received = false;
  bool map_updated = false;

  smarc_msgs::msg::GeofenceStatusStamped geofence_message;
  bool geofence_received = false;
  bool geofence_updated = false;

  //Error buckets
  ErrorBucket odom_value_error_bucket;
  ErrorBucket odom_time_error_bucket;

  ErrorBucket ekfNav_value_error_bucket;
  ErrorBucket ekfNav_time_error_bucket;

  ErrorBucket captainState_value_error_bucket;
  ErrorBucket captainState_time_error_bucket;

  ErrorBucket map_value_error_bucket;
  ErrorBucket map_time_error_bucket;

  ErrorBucket geofence_time_error_bucket;


  // -----------------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry msg) {
    //std::cout << "Odom callback \n";
    odom_message = msg;
    odom_received = true;
    odom_updated = true;
  }

  // -----------------------------------------------------------------------
  void sbgNavCallback(const sbg_driver::msg::SbgEkfNav msg) {
    //std::cout << "SBG NAV callback \n";
    ekfNav_message = msg;
    ekfNav_received = true;
    ekfNav_updated = true;
  }
  // -----------------------------------------------------------------------
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid msg) {
    //std::cout << "Ocupancy grid callback \n";
    map_message = msg;
  }

  void captainStateCallback(const evolo_msgs::msg::CaptainState msg) {
    //std::cout << "Captain state callback \n";
    captain_state_msg = msg;
    captain_received = true;
    captain_updated = true;
  }

  void geofenceCallback(const smarc_msgs::msg::GeofenceStatusStamped msg) {
    //std::cout << "Geofence callback \n";
    geofence_message = msg;
    geofence_received = true;
    geofence_updated = true;
  }

  // -----------------------------------------------------------------------
  void timer_callback() {
    
    //Prevent error on startup
    if(!odom_received ||
      !ekfNav_received) //||
      //!map_received ||
      //!captain_received)
    {
      std::cout << "NOT READY" << std::endl;
      std_msgs::msg::Int8 msg;
      msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_WAITING;
      _health_pub->publish(msg);
      return;
    }
    

    //Timeout checks
    bool timeout_error = false;
    if(!odom_time_error_bucket.update(odom_updated)) { timeout_error = true; std::cout << "TIMEOUT_ERROR: Odometry" << std::endl; }
    if(!ekfNav_time_error_bucket.update(ekfNav_updated))  { timeout_error = true; std::cout << "TIMEOUT_ERROR: EkfNav" << std::endl; }
    if(!geofence_time_error_bucket.update(geofence_updated))  { timeout_error = true; std::cout << "TIMEOUT_ERROR: Geofence" << std::endl; }
    //if(!map_time_error_bucket.update(map_updated)  { timeout_error = true; std::cout << "TIMEOUT_ERROR: Odometry" << std::endl; }
    //if(!captainState_time_error_bucket.update(captain_updated)) { timeout_error = true; std::cout << "TIMEOUT_ERROR: CaptainState" << std::endl; }
        
    //Timeout error
    if(timeout_error) {
      //Timeout error
      std::cout << "Timout error" << std::endl;
      std_msgs::msg::Int8 msg;
      msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //ERROR
      _health_pub->publish(msg);
      return;
    }
    
    std_msgs::msg::Int8 health_msg;
    health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_READY; //All good

    //Data checks
    //Outside geofence
    if (geofence_message.status == geofence_message.STATUS_OUTSIDE) {
      health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
      std::cout << "Outsude Geofence" << std::endl;
    }
    else if (geofence_message.status == smarc_msgs::msg::GeofenceStatusStamped::STATUS_INACTIVE) {
      float vx = odom_message.twist.twist.linear.x;
      float vy = odom_message.twist.twist.linear.y;
      float speed = sqrt(vx*vx + vy*vy);
      if(speed > 1.0) {
        std::cout << "Too fast without active geofence" << std::endl;
        health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
      }
    }


    //std::cout << "All is good" << std::endl;
    
    _health_pub->publish(health_msg);


    //reset variables
    odom_updated = false;
    ekfNav_updated = false;
    map_updated = false;
    captain_updated = false;
    geofence_updated = false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthChecker>());
  rclcpp::shutdown();

  return 0;
}
