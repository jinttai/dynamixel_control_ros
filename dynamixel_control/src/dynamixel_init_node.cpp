#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <tgmath.h>
#include <cmath>
#include <vector>
#include <string>

// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"

// Custom message / service definitions
#include "dynamixel_msg/msg/dxl_control.hpp"
#include "dynamixel_msg/srv/dxl_state.hpp"

// Device and protocol settings
#define DEVICENAME           "/dev/ttyUSB0"
#define BAUDRATE             57600
#define PROTOCOL_VERSION     2.0

// Arbitrary settings
#define MOVE_TIME            1000
#define COMM_TX_FAIL         -1001

// Control table addresses (example for XM430)
#define ADDR_XM_GOAL_POSITION        116
#define ADDR_XM_PRESENT_POSITION     132
#define ADDR_XM_GOAL_SPEED           104
#define ADDR_XM_GOAL_CURRENT         102
#define ADDR_XM_PRESENT_SPEED        128
#define ADDR_XM_TORQUE_ENABLE        64
#define ADDR_XM_OPERATING_MODE       11
#define ADDR_XM_MAX_POSITION         48
#define ADDR_XM_MIN_POSITION         52
#define ADDR_XM_VELOCITY_PROFILE     112
#define ADDR_XM_PROFILE_ACCELERATION 108

// Example limit values
#define MAX_POSITION        4095
#define MIN_POSITION        0

const std::array<uint8_t, 3> dxl_ids_ = {1, 2, 3};


class DynamixelInitNode : public rclcpp::Node
{
public:
  DynamixelInitNode()
  : Node("dynamixel_init_node"), run_once_flag_(0)
  {
    // 1) Initialize PortHandler and PacketHandler
    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // 2) Perform basic Dynamixel initialization (open port, set baudrate, etc.)
    initializeDynamixel();

    // 3) Create GroupBulkWrite and GroupBulkRead objects
    group_bulk_write_ = std::make_unique<dynamixel::GroupBulkWrite>(port_handler_, packet_handler_);
    group_bulk_read_  = std::make_unique<dynamixel::GroupBulkRead>(port_handler_, packet_handler_);

    // 4) Create a service server (dxl_state)
    service_ = this->create_service<dynamixel_msg::srv::DxlState>(
      "dxl_state",
      std::bind(&DynamixelInitNode::stateCallback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    // 5) Create a subscriber (control_value)
    subscription_ = this->create_subscription<dynamixel_msg::msg::DxlControl>(
      "control_value",
      10,
      std::bind(&DynamixelInitNode::controlCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "DynamixelInitNode constructed.");
  }

private:
  // ----------------------------------------------------------------------------
  // Basic Dynamixel initialization
  // ----------------------------------------------------------------------------
  void initializeDynamixel()
  {
    // Open the serial port
    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to open the port.");
    }

    // Set the baudrate
    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");
    }

    // Write some default configuration to each Dynamixel ID
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for (auto id : dxl_ids_) {
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_XM_MAX_POSITION, MAX_POSITION, &dxl_error);
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_XM_MIN_POSITION, MIN_POSITION, &dxl_error);
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_XM_VELOCITY_PROFILE, MOVE_TIME, &dxl_error);
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, ADDR_XM_PROFILE_ACCELERATION, 0, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "Initialization for ID %d failed. CommResult:%d, Error:%d",
                    id, dxl_comm_result, dxl_error);
      }
    }
  }

  // ----------------------------------------------------------------------------
  // Set each operating mode
  // ----------------------------------------------------------------------------
  void setCurrentMode()
  {
    // Operating mode = 0 (Current)
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    for (auto id : dxl_ids_) {
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_OPERATING_MODE, 0, &dxl_error);
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "[ID:%d] Setting Current Mode failed. CommResult:%d, Error:%d",
                    id, dxl_comm_result, dxl_error);
      }
    }
    RCLCPP_INFO(this->get_logger(), "All IDs set to Current Mode (Torque).");
  }

  void setSpeedMode()
  {
    // Operating mode = 1 (Velocity)
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    for (auto id : dxl_ids_) {
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_OPERATING_MODE, 1, &dxl_error);
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "[ID:%d] Setting Speed Mode failed. CommResult:%d, Error:%d",
                    id, dxl_comm_result, dxl_error);
      }
    }
    RCLCPP_INFO(this->get_logger(), "All IDs set to Speed Mode.");
  }

  void setPositionMode()
  {
    // Operating mode = 3 (Position)
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    for (auto id : dxl_ids_) {
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_OPERATING_MODE, 3, &dxl_error);
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "[ID:%d] Setting Position Mode failed. CommResult:%d, Error:%d",
                    id, dxl_comm_result, dxl_error);
      }
    }
    RCLCPP_INFO(this->get_logger(), "All IDs set to Position Mode.");
  }

  // ----------------------------------------------------------------------------
  // Control commands for each mode
  // ----------------------------------------------------------------------------
  void controlCurrentMode(const std::vector<float> &control_value)
  {
    const float TORQUE_TO_TICK = 2.3f / 4.1f / 0.269f;

    std::vector<int> current_bits;
    current_bits.reserve(control_value.size());

    for (auto val : control_value) {
      float scaled_val = val * TORQUE_TO_TICK;
      int16_t rounded_val = static_cast<int16_t>(std::round(scaled_val));
      current_bits.push_back(rounded_val);
    }

    // Clear old parameters in BulkWrite
    group_bulk_write_->clearParam();

    // Write current for each ID
    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      if (i < current_bits.size()) {
        uint8_t param_goal_current[2];
        param_goal_current[0] = DXL_LOBYTE(current_bits[i]);
        param_goal_current[1] = DXL_HIBYTE(current_bits[i]);

        bool addparam_result = group_bulk_write_->addParam(
          dxl_ids_[i],
          ADDR_XM_GOAL_CURRENT,
          2,
          param_goal_current
        );
        if (!addparam_result) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to addParam for ID %d in Current Mode.",
                       dxl_ids_[i]);
        }
      }
    }

    // Send the BulkWrite packet
    int tx_result = group_bulk_write_->txPacket();
    if (tx_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "BulkWrite txPacket failed: %d", tx_result);
    } else {
      RCLCPP_INFO(this->get_logger(), "Current Mode command sent via BulkWrite.");
    }
  }

  void controlSpeedMode(const std::vector<float> &control_value)
  {
    const float SPEED_TO_TICK = 1.0f / (0.229f * 6.0f);

    std::vector<int> speed_bits;
    speed_bits.reserve(control_value.size());

    for (auto val : control_value) {
      float scaled_val = val * SPEED_TO_TICK;
      int32_t rounded_val = static_cast<int32_t>(std::round(scaled_val));
      speed_bits.push_back(rounded_val);
    }

    group_bulk_write_->clearParam();

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      if (i < speed_bits.size()) {
        uint8_t param_goal_speed[4];
        param_goal_speed[0] = DXL_LOBYTE(DXL_LOWORD(speed_bits[i]));
        param_goal_speed[1] = DXL_HIBYTE(DXL_LOWORD(speed_bits[i]));
        param_goal_speed[2] = DXL_LOBYTE(DXL_HIWORD(speed_bits[i]));
        param_goal_speed[3] = DXL_HIBYTE(DXL_HIWORD(speed_bits[i]));

        bool addparam_result = group_bulk_write_->addParam(
          dxl_ids_[i],
          ADDR_XM_GOAL_SPEED,
          4,
          param_goal_speed
        );
        if (!addparam_result) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to addParam for ID %d in Speed Mode.",
                       dxl_ids_[i]);
        }
      }
    }

    int tx_result = group_bulk_write_->txPacket();
    if (tx_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "BulkWrite txPacket failed: %d", tx_result);
    } else {
      RCLCPP_INFO(this->get_logger(), "Speed Mode command sent via BulkWrite.");
    }
  }

  void controlPositionMode(const std::vector<float> &control_value)
  {
    const float POSITION_TO_TICK = 1.0f / 0.088f;

    std::vector<int> position_bits;
    position_bits.reserve(control_value.size());

    for (auto val : control_value) {
      float scaled_val = val * POSITION_TO_TICK;
      int32_t rounded_val = static_cast<int32_t>(std::round(scaled_val));
      position_bits.push_back(rounded_val);
    }

    group_bulk_write_->clearParam();

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      if (i < position_bits.size()) {
        uint8_t param_goal_position[4];
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position_bits[i]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position_bits[i]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position_bits[i]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position_bits[i]));

        bool addparam_result = group_bulk_write_->addParam(
          dxl_ids_[i],
          ADDR_XM_GOAL_POSITION,
          4,
          param_goal_position
        );
        if (!addparam_result) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to addParam for ID %d in Position Mode.",
                       dxl_ids_[i]);
        }
      }
    }

    int tx_result = group_bulk_write_->txPacket();
    if (tx_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "BulkWrite txPacket failed: %d", tx_result);
    } else {
      RCLCPP_INFO(this->get_logger(), "Position Mode command sent via BulkWrite.");
    }
  }

  // ----------------------------------------------------------------------------
  // Service callback: returns current angles and velocities
  // ----------------------------------------------------------------------------
  void stateCallback(const std::shared_ptr<dynamixel_msg::srv::DxlState::Request> request,
                     std::shared_ptr<dynamixel_msg::srv::DxlState::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "DxlState service called.");

    // Clear old parameters in GroupBulkRead
    group_bulk_read_->clearParam();
    for (auto id : dxl_ids_) {
      group_bulk_read_->addParam(id, ADDR_XM_PRESENT_POSITION, 4);
      group_bulk_read_->addParam(id, ADDR_XM_PRESENT_SPEED,    4);
    }

    // Read from Dynamixels
    int dxl_comm_result = group_bulk_read_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get present position & speed! CommResult:%d",
                   dxl_comm_result);
    }

    // Prepare response vectors
    response->joint_angle.clear();
    response->joint_angular_velocity.clear();
    response->joint_angle.reserve(dxl_ids_.size());
    response->joint_angular_velocity.reserve(dxl_ids_.size());

    // Example conversion constants
    const float TICK_TO_DEG = 0.088f;
    const float TICK_TO_RPM = 0.229f;

    for (auto id : dxl_ids_) {
      int32_t present_position = group_bulk_read_->getData(id, ADDR_XM_PRESENT_POSITION, 4);
      int32_t present_velocity = group_bulk_read_->getData(id, ADDR_XM_PRESENT_SPEED,    4);

      // Convert ticks to degrees
      float angle_deg = present_position * TICK_TO_DEG;
      float velocity_deg_s = present_velocity * (TICK_TO_RPM * 6.0f);

      response->joint_angle.push_back(angle_deg);
      response->joint_angular_velocity.push_back(velocity_deg_s);

      RCLCPP_INFO(this->get_logger(),
                  "[ID:%d] raw_pos:%d -> %.2f deg, raw_vel:%d -> %.2f deg/s",
                  id, present_position, angle_deg,
                  present_velocity, velocity_deg_s);
    }
  }

  // ----------------------------------------------------------------------------
  // Turn off torque on all Dynamixels
  // ----------------------------------------------------------------------------
  void terminateDynamixel()
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for (auto id : dxl_ids_) {
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_XM_TORQUE_ENABLE, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to disable torque [ID:%d]. CommResult:%d, Error:%d",
                    id, dxl_comm_result, dxl_error);
      }
    }
    RCLCPP_INFO(this->get_logger(), "All Dynamixels' torque disabled.");
  }

  // ----------------------------------------------------------------------------
  // Subscriber callback: receives control commands (operate_mode, control_value)
  // ----------------------------------------------------------------------------
  void controlCallback(const dynamixel_msg::msg::DxlControl::SharedPtr msg)
  {
    int operate_mode = msg->operate_mode;
    const std::vector<float> &control_value = msg->control_value;

    RCLCPP_INFO(this->get_logger(), "Received operate_mode: %d", operate_mode);

    switch (operate_mode) {
      case 0: // Current (Torque) control mode
      {
        if (run_once_flag_ == 0) {
          setCurrentMode();
          controlCurrentMode(control_value);
          run_once_flag_ = 1;
          RCLCPP_INFO(this->get_logger(), "Current control mode started.");
        } else {
          controlCurrentMode(control_value);
          RCLCPP_INFO(this->get_logger(), "Current control mode updated.");
        }
        break;
      }
      case 1: // Speed control mode
      {
        if (run_once_flag_ == 0) {
          setSpeedMode();
          controlSpeedMode(control_value);
          run_once_flag_ = 1;
          RCLCPP_INFO(this->get_logger(), "Speed control mode started.");
        } else {
          controlSpeedMode(control_value);
          RCLCPP_INFO(this->get_logger(), "Speed control mode updated.");
        }
        break;
      }
      case 3: // Position control mode
      {
        if (run_once_flag_ == 0) {
          setPositionMode();
          controlPositionMode(control_value);
          run_once_flag_ = 1;
          RCLCPP_INFO(this->get_logger(), "Position control mode started.");
        } else {
          controlPositionMode(control_value);
          RCLCPP_INFO(this->get_logger(), "Position control mode updated.");
        }
        break;
      }
      case 4: // Stop (torque off)
      {
        run_once_flag_ = 0;
        RCLCPP_INFO(this->get_logger(), "Stop command received.");
        terminateDynamixel();
        break;
      }
      default:
      {
        RCLCPP_WARN(this->get_logger(), "Unknown operate_mode: %d", operate_mode);
        break;
      }
    }
  }

private:
  // ----------------------------------------------------------------------------
  // Member variables
  // ----------------------------------------------------------------------------
  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;
  std::unique_ptr<dynamixel::GroupBulkWrite> group_bulk_write_;
  std::unique_ptr<dynamixel::GroupBulkRead>  group_bulk_read_;

  // Service and subscriber
  rclcpp::Service<dynamixel_msg::srv::DxlState>::SharedPtr service_;
  rclcpp::Subscription<dynamixel_msg::msg::DxlControl>::SharedPtr subscription_;

  // A flag to indicate whether initial mode setting is done (0 = not done, 1 = done)
  int run_once_flag_;
};

// ----------------------------------------------------------------------------
// main function
// ----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting DynamixelInitNode...");

  auto node = std::make_shared<DynamixelInitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
