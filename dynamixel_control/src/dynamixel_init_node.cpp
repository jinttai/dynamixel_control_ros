#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <tgmath.h>
#include <cmath>
#include <vector>
#include <string>

// Include the dynamixel headers
#include "dynamixel_sdk/dynamixel_sdk.h"
// Include the custom message
#include "dynamixel_msg/msg/dxl_control.hpp"

// Define the device name, baudrate, and protocol
#define DEVICENAME                      "/dev/ttyUSB0"
#define BAUDRATE                        57600
#define PROTOCOL_VERSION                2.0
#define MOVE_TIME                       1000
#define COMM_TX_FAIL                    -1001

// Control table addresses
#define ADDR_XM_GOAL_POSITION           116
#define ADDR_XM_PRESENT_POSITION        132  
#define ADDR_XM_GOAL_SPEED              104
#define ADDR_XM_GOAL_CURRENT            102
#define ADDR_XM_PRESENT_SPEED           128
#define ADDR_XM_PRESENT_LOAD            126
#define ADDR_XM_POSITION_P_GAIN         84       
#define ADDR_XM_POSITION_I_GAIN         82
#define ADDR_XM_POSITION_D_GAIN         80
#define ADDR_XM_VELOCITY_P_GAIN         78       
#define ADDR_XM_VELOCITY_I_GAIN         76
#define ADDR_XM_TORQUE_ENABLE           64
#define ADDR_XM_MAX_POSITION            48
#define ADDR_XM_MIN_POSITION            52
#define ADDR_XM_VELOCITY_LIMIT          44
#define ADDR_XM_VELOCITY_PROFILE        112
#define ADDR_XM_CURRENT_LIMIT           38
#define ADDR_XM_RETURN_DELAY            9
#define ADDR_XM_DRIVE_MODE              10
#define ADDR_XM_OPERATING_MODE          11
#define ADDR_XM_PWM_LIMIT               36
#define ADDR_XM_GOAL_PWM                100
#define ADDR_XM_PROFILE_ACCELERATION    108

#define CURRENT_LIMIT                   1000
#define SPEED_P_GAIN                    100
#define MAX_POSITION                    4095
#define MIN_POSITION                    0

class Dynamixel_Init_Node : public rclcpp::Node
{
public:
  Dynamixel_Init_Node()
  : Node("dynamixel_init_node"), run_once_flag(0)
  {
    // 포트핸들러, 패킷핸들러 초기화
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    // Dynamixel 초기화
    initialize_dynamixel();

    // groupBulkWrite를 now 초기화 (포트핸들러, 패킷핸들러가 유효한 상태)
    groupBulkWrite = std::make_unique<dynamixel::GroupBulkWrite>(portHandler, packetHandler);

    subscription_ = this->create_subscription<dynamixel_msg::msg::DxlControl>(
      "control_value",
       10, 
       std::bind(&Dynamixel_Init_Node::control_callback, this, std::placeholders::_1)
    );
  }

private:
  void initialize_dynamixel()
  {
    // 포트 오픈
    if (!portHandler->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to open the port.");
    }

    // 보드레이트 설정
    if (!portHandler->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");
    }

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    // 1번 ID 초기화
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_XM_MAX_POSITION, MAX_POSITION, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_XM_MIN_POSITION, MIN_POSITION, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_XM_VELOCITY_PROFILE, MOVE_TIME, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_XM_PROFILE_ACCELERATION, 0, &dxl_error);
    

    // 2번 ID 초기화
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_XM_MAX_POSITION, MAX_POSITION, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_XM_MIN_POSITION, MIN_POSITION, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_XM_VELOCITY_PROFILE, MOVE_TIME, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_XM_PROFILE_ACCELERATION, 0, &dxl_error);
    

    // 3번 ID 초기화
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_XM_MAX_POSITION, MAX_POSITION, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_XM_MIN_POSITION, MIN_POSITION, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_XM_VELOCITY_PROFILE, MOVE_TIME, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_XM_PROFILE_ACCELERATION, 0, &dxl_error);
    
  }

  void initial_current_mode()
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_OPERATING_MODE, 0, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_OPERATING_MODE, 0, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_OPERATING_MODE, 0, &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    std::cout << "dxl_comm_result: " << dxl_comm_result << std::endl;
  }

  void initial_speed_mode()
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_OPERATING_MODE, 1, &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    std::cout << "dxl_comm_result: " << dxl_comm_result << std::endl;
  }

  void initial_position_mode()
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_OPERATING_MODE, 3, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_OPERATING_MODE, 3, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_OPERATING_MODE, 3, &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_TORQUE_ENABLE, 1, &dxl_error);
    std::cout << "dxl_comm_result: " << dxl_comm_result << std::endl;

  }

  void control_current_mode(const std::vector<float>& control_value)
  {
    bool dxl_addparam_result = false;

    std::vector<int> current_bits;
    current_bits.reserve(control_value.size());
    for (auto val : control_value) {
      float scaled_val = val / 4.1 * 2.3 / 269 * 1000;
      int16_t rounded_val = static_cast<int>(std::round(scaled_val));
      current_bits.push_back(rounded_val);
    }

    uint8_t param_goal_TAU_1[2];
    uint8_t param_goal_TAU_2[2];
    uint8_t param_goal_TAU_3[2];

    param_goal_TAU_1[0] = DXL_LOBYTE(current_bits[0]);
    param_goal_TAU_1[1] = DXL_HIBYTE(current_bits[0]);

    param_goal_TAU_2[0] = DXL_LOBYTE(current_bits[1]);
    param_goal_TAU_2[1] = DXL_HIBYTE(current_bits[1]);

    param_goal_TAU_3[0] = DXL_LOBYTE(current_bits[2]);
    param_goal_TAU_3[1] = DXL_HIBYTE(current_bits[2]);

    groupBulkWrite->clearParam();
    groupBulkWrite->addParam(1, ADDR_XM_GOAL_CURRENT, 2, param_goal_TAU_1);
    groupBulkWrite->addParam(2, ADDR_XM_GOAL_CURRENT, 2, param_goal_TAU_2);
    groupBulkWrite->addParam(3, ADDR_XM_GOAL_CURRENT, 2, param_goal_TAU_3);

    groupBulkWrite->txPacket();
  }

  void control_speed_mode(const std::vector<float>& control_value)
  {
    bool dxl_addparam_result = false;

    std::vector<int> speed_bits;
    speed_bits.reserve(control_value.size());
    for (auto val : control_value) {
      float scaled_val = val / 229 * 1000;
      int32_t rounded_val = static_cast<int>(std::round(scaled_val));
      speed_bits.push_back(rounded_val);
    }

    uint8_t param_goal_speed_1[4];
    uint8_t param_goal_speed_2[4];
    uint8_t param_goal_speed_3[4];

    param_goal_speed_1[0] = DXL_LOBYTE(DXL_LOWORD(speed_bits[0]));
    param_goal_speed_1[1] = DXL_HIBYTE(DXL_LOWORD(speed_bits[0]));
    param_goal_speed_1[2] = DXL_LOBYTE(DXL_HIWORD(speed_bits[0]));
    param_goal_speed_1[3] = DXL_HIBYTE(DXL_HIWORD(speed_bits[0]));

    param_goal_speed_2[0] = DXL_LOBYTE(DXL_LOWORD(speed_bits[1]));
    param_goal_speed_2[1] = DXL_HIBYTE(DXL_LOWORD(speed_bits[1]));
    param_goal_speed_2[2] = DXL_LOBYTE(DXL_HIWORD(speed_bits[1]));
    param_goal_speed_2[3] = DXL_HIBYTE(DXL_HIWORD(speed_bits[1]));

    param_goal_speed_3[0] = DXL_LOBYTE(DXL_LOWORD(speed_bits[2]));
    param_goal_speed_3[1] = DXL_HIBYTE(DXL_LOWORD(speed_bits[2]));
    param_goal_speed_3[2] = DXL_LOBYTE(DXL_HIWORD(speed_bits[2]));
    param_goal_speed_3[3] = DXL_HIBYTE(DXL_HIWORD(speed_bits[2]));

    groupBulkWrite->clearParam();
    groupBulkWrite->addParam(1, ADDR_XM_GOAL_SPEED, 4, param_goal_speed_1);
    groupBulkWrite->addParam(2, ADDR_XM_GOAL_SPEED, 4, param_goal_speed_2);
    groupBulkWrite->addParam(3, ADDR_XM_GOAL_SPEED, 4, param_goal_speed_3);

    groupBulkWrite->txPacket();
  }

  void control_position_mode(const std::vector<float>& control_value)
  {
    bool dxl_addparam_result = false;
    uint8_t param_goal_position_1[4];
    uint8_t param_goal_position_2[4];
    uint8_t param_goal_position_3[4];

    std::vector<int> position_bits;
    position_bits.reserve(control_value.size());
    for (auto val : control_value) {
      float scaled_val = val / 88 * 1000;
      int32_t rounded_val = static_cast<int>(std::round(scaled_val));
      position_bits.push_back(rounded_val);
    }

    param_goal_position_1[0] = DXL_LOBYTE(DXL_LOWORD(position_bits[0]));
    param_goal_position_1[1] = DXL_HIBYTE(DXL_LOWORD(position_bits[0]));
    param_goal_position_1[2] = DXL_LOBYTE(DXL_HIWORD(position_bits[0]));
    param_goal_position_1[3] = DXL_HIBYTE(DXL_HIWORD(position_bits[0]));

    param_goal_position_2[0] = DXL_LOBYTE(DXL_LOWORD(position_bits[1]));
    param_goal_position_2[1] = DXL_HIBYTE(DXL_LOWORD(position_bits[1]));
    param_goal_position_2[2] = DXL_LOBYTE(DXL_HIWORD(position_bits[1]));
    param_goal_position_2[3] = DXL_HIBYTE(DXL_HIWORD(position_bits[1]));

    param_goal_position_3[0] = DXL_LOBYTE(DXL_LOWORD(position_bits[2]));
    param_goal_position_3[1] = DXL_HIBYTE(DXL_LOWORD(position_bits[2]));
    param_goal_position_3[2] = DXL_LOBYTE(DXL_HIWORD(position_bits[2]));
    param_goal_position_3[3] = DXL_HIBYTE(DXL_HIWORD(position_bits[2]));

    groupBulkWrite->clearParam();
    groupBulkWrite->addParam(1, ADDR_XM_GOAL_POSITION, 4, param_goal_position_1);
    groupBulkWrite->addParam(2, ADDR_XM_GOAL_POSITION, 4, param_goal_position_2);
    groupBulkWrite->addParam(3, ADDR_XM_GOAL_POSITION, 4, param_goal_position_3);

    groupBulkWrite->txPacket();
  }

  void terminate_dynamixel()
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_XM_TORQUE_ENABLE, 0, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_XM_TORQUE_ENABLE, 0, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_XM_TORQUE_ENABLE, 0, &dxl_error);
  }

  void control_callback(const dynamixel_msg::msg::DxlControl::SharedPtr msg)
  {
    int operate_mode = msg->operate_mode;
    const std::vector<float>& control_value = msg->control_value;
    RCLCPP_INFO(this->get_logger(), "Received operate_mode: %d", operate_mode);

    switch (operate_mode){
      case 0: // current(torque) control mode
        if(run_once_flag == 0){
          initial_current_mode();
          control_current_mode(control_value);
          run_once_flag = 1;
          RCLCPP_INFO(this->get_logger(), "Current control mode started.");

        }
        else{
          control_current_mode(control_value);
          RCLCPP_INFO(this->get_logger(), "Current control mode running.");
        }
        break;
      case 1: // speed control mode
        if(run_once_flag == 0){
          initial_speed_mode();
          control_speed_mode(control_value);
          run_once_flag = 1;
          RCLCPP_INFO(this->get_logger(), "Speed control mode started.");
        }
        else{
          control_speed_mode(control_value);
          RCLCPP_INFO(this->get_logger(), "Speed control mode running.");
        }
        break;
      case 3: // position control mode
        if(run_once_flag == 0){
          initial_position_mode();
          control_position_mode(control_value);
          run_once_flag = 1;
          RCLCPP_INFO(this->get_logger(), "Position control mode started.");
        }
        else{
          control_position_mode(control_value);
          RCLCPP_INFO(this->get_logger(), "Position control mode running.");
        }
        break;
      case 4: // stop
        run_once_flag = 0;
        RCLCPP_INFO(this->get_logger(), "Stop control mode.");
        terminate_dynamixel();
        break;
    }
  }

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  std::unique_ptr<dynamixel::GroupBulkWrite> groupBulkWrite;

  rclcpp::Subscription<dynamixel_msg::msg::DxlControl>::SharedPtr subscription_;
  int run_once_flag;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  
  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting dynamixel node initialization...");

  auto node = std::make_shared<Dynamixel_Init_Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
