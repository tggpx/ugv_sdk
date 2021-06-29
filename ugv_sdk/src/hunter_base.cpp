#include "ugv_sdk/hunter/hunter_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>

#include "stopwatch.hpp"

namespace westonrobot {

void HunterBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  SendModeCtl();
  SetParkMode();
  SendMotionCmd(cmd_count++);
}

void HunterBase::SendMotionCmd(uint8_t count) {
  // motion control message
  AgxMessage m_msg;
  m_msg.type = AgxMsgMotionCommand;
  memset(m_msg.body.motion_command_msg.raw, 0, 8);
  /*if (can_connected_)
    m_msg.body.motion_control_msg.data.cmd.control_mode = CTRL_MODE_CMD_CAN;
  else if (serial_connected_)
    m_msg.body.motion_cmd_msg.data.cmd.control_mode = CTRL_MODE_CMD_UART*/
  ;
  motion_cmd_mutex_.lock();
  m_msg.body.motion_command_msg.cmd.linear_velocity.high_byte =
      current_motion_cmd_.linear_velocity_height_byte;
  m_msg.body.motion_command_msg.cmd.linear_velocity.low_byte =
      current_motion_cmd_.linear_velocity_low_byte;
  m_msg.body.motion_command_msg.cmd.steering_angle.high_byte =
      current_motion_cmd_.angular_velocity_height_byte;
  m_msg.body.motion_command_msg.cmd.steering_angle.low_byte =
      current_motion_cmd_.angular_velocity_low_byte;
  motion_cmd_mutex_.unlock();

  // send to can bus
  if (can_connected_) {
    can_frame m_frame;
    EncodeCanFrame(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  }
}

HunterState HunterBase::GetHunterState() {
  std::lock_guard<std::mutex> guard(hunter_state_mutex_);
  return hunter_state_;
}

void HunterBase::SetMotionCommand(
    double linear_vel, double angular_vel, double steering_angle,
    HunterMotionCmd::FaultClearFlag fault_clr_flag) {
  // make sure cmd thread is started before attempting to send commands
  if (!cmd_thread_started_) StartCmdThread();

  if (linear_vel < HunterMotionCmd::min_linear_velocity)
    linear_vel = HunterMotionCmd::min_linear_velocity;
  if (linear_vel > HunterMotionCmd::max_linear_velocity)
    linear_vel = HunterMotionCmd::max_linear_velocity;
  if (steering_angle < HunterMotionCmd::min_steering_angle)
    steering_angle = HunterMotionCmd::min_steering_angle;
  if (steering_angle > HunterMotionCmd::max_steering_angle)
    steering_angle = HunterMotionCmd::max_steering_angle;

  std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  current_motion_cmd_.linear_velocity_height_byte =
      static_cast<int16_t>(linear_vel * 1000) >> 8;
  current_motion_cmd_.linear_velocity_low_byte =
      static_cast<int16_t>(linear_vel * 1000) & 0xff;
  current_motion_cmd_.angular_velocity_height_byte =
      static_cast<int16_t>(steering_angle * 1000) >> 8;
  current_motion_cmd_.angular_velocity_low_byte =
      static_cast<int16_t>(steering_angle * 1000) & 0xff;
  current_motion_cmd_.fault_clear_flag = fault_clr_flag;

  FeedCmdTimeoutWatchdog();
}

void HunterBase::SendModeCtl() {
  AgxMessage m_msg;
  m_msg.type = AgxMsgCtrlModeSelect;
  memset(m_msg.body.ctrl_mode_select_msg.raw, 0, 8);
  m_msg.body.ctrl_mode_select_msg.cmd.control_mode = CTRL_MODE_CMD_CAN;

  if (can_connected_) {
    // send to can bus
    can_frame m_frame;
    EncodeCanFrame(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  } else {
  }
}

void HunterBase::SetParkMode() {
  AgxMessage m_msg;
  m_msg.type = AgxMsgParkModeSelect;
  bool flag = current_motion_cmd_.linear_velocity_height_byte ||
              current_motion_cmd_.linear_velocity_low_byte  ||
              current_motion_cmd_.angular_velocity_height_byte  ||
              current_motion_cmd_.angular_velocity_low_byte;
  if (flag) {
    pack_mode_cmd_mutex_.lock();
    m_msg.body.park_control_msg.cmd.parking_mode = 0x00;
    pack_mode_cmd_mutex_.unlock();
  } else {
    pack_mode_cmd_mutex_.lock();
    m_msg.body.park_control_msg.cmd.parking_mode = 0x01;
    pack_mode_cmd_mutex_.unlock();
  }
  m_msg.body.park_control_msg.cmd.reserved0 = 0;
  m_msg.body.park_control_msg.cmd.reserved1 = 0;
  m_msg.body.park_control_msg.cmd.reserved2 = 0;
  m_msg.body.park_control_msg.cmd.reserved3 = 0;
  m_msg.body.park_control_msg.cmd.reserved4 = 0;
  m_msg.body.park_control_msg.cmd.reserved5 = 0;
  m_msg.body.park_control_msg.cmd.reserved6 = 0;
  if (can_connected_) {
    // send to can bus
    can_frame m_frame;
    EncodeCanFrame(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  } else {
  }
}

void HunterBase::ParseCANFrame(can_frame *rx_frame) {
  // update robot state with new frame
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  NewStatusMsgReceivedCallback(status_msg);
}

void HunterBase::NewStatusMsgReceivedCallback(const AgxMessage &msg) {
  // std::cout << "new status msg received" << std::endl;
  std::lock_guard<std::mutex> guard(hunter_state_mutex_);
  UpdateHunterState(msg, hunter_state_);
}

void HunterBase::UpdateHunterState(const AgxMessage &status_msg,
                                   HunterState &state) {
  switch (status_msg.type) {
    case AgxMsgSystemState: {
      // std::cout << "system status feedback received" << std::endl;
      const SystemStateMessage &msg = status_msg.body.system_state_msg;
      state.control_mode = msg.state.control_mode;
      state.base_state = msg.state.vehicle_state;
      state.battery_voltage =
          (static_cast<uint16_t>(msg.state.battery_voltage.low_byte) |
           static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8) /
          10.0;
      state.fault_code = msg.state.fault_code;
      state.park_mode = msg.state.park_mode;
      break;
    }
    case AgxMsgMotionState: {
      // std::cout << "motion control feedback received" << std::endl;
      const MotionStateMessage &msg = status_msg.body.motion_state_msg;
      state.linear_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.state.linear_velocity.low_byte) |
              static_cast<uint16_t>(msg.state.linear_velocity.high_byte) << 8) /
          1000.0;
      //std::cout << state.linear_velocity << std::endl;
      state.steering_angle =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.state.steering_angle.low_byte) |
              static_cast<uint16_t>(msg.state.steering_angle.high_byte)
                  << 8) /
          1000.0;
      break;
    }
    case AgxMsgActuatorLSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      const ActuatorLSStateMessage &msg = status_msg.body.actuator_ls_state_msg;
      for (int i = 0; i < 2; ++i) {
        state.actuator_states[msg.motor_id].driver_voltage =
            (static_cast<uint16_t>(msg.data.state.driver_voltage.low_byte) |
             static_cast<uint16_t>(msg.data.state.driver_voltage.high_byte)
                 << 8) /
            10.0;
        state.actuator_states[msg.motor_id]
            .driver_temperature = static_cast<int16_t>(
            static_cast<uint16_t>(msg.data.state.driver_temperature.low_byte) |
            static_cast<uint16_t>(msg.data.state.driver_temperature.high_byte)
                << 8);
        state.actuator_states[msg.motor_id].motor_temperature =
            msg.data.state.motor_temperature;
        state.actuator_states[msg.motor_id].driver_state =
            msg.data.state.driver_state;
      }
      break;
    }
    case AgxMsgActuatorHSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      const ActuatorHSStateMessage &msg = status_msg.body.actuator_hs_state_msg;
      for (int i = 0; i < 2; ++i) {
        state.actuator_states[msg.motor_id].motor_rpm =
            (static_cast<uint16_t>(msg.data.state.rpm.low_byte) |
             static_cast<uint16_t>(msg.data.state.rpm.high_byte)
                 << 8);
        state.actuator_states[msg.motor_id]
            .motor_current = static_cast<int16_t>(
            static_cast<uint16_t>(msg.data.state.current.low_byte) |
            static_cast<uint16_t>(msg.data.state.current.high_byte)
                << 8) / 10.0;
        state.actuator_states[msg.motor_id].motor_pulses = static_cast<int32_t>(
            (static_cast<uint32_t>(msg.data.state.pulse_count.lsb)) |
            (static_cast<uint32_t>(msg.data.state.pulse_count.low_byte) << 8) |
            (static_cast<uint32_t>(msg.data.state.pulse_count.high_byte) << 16) |
            (static_cast<uint32_t>(msg.data.state.pulse_count.msb) << 24));
      }
      break;
    }
    case AgxMsgOdometry: {
      // std::cout << "Odometer msg feedback received" << std::endl;
      const OdometryMessage &msg = status_msg.body.odometry_msg;
      state.right_odometry = static_cast<int32_t>(
          (static_cast<uint32_t>(msg.state.right_wheel.lsb)) |
          (static_cast<uint32_t>(msg.state.right_wheel.low_byte) << 8) |
          (static_cast<uint32_t>(msg.state.right_wheel.high_byte) << 16) |
          (static_cast<uint32_t>(msg.state.right_wheel.msb) << 24));
      state.left_odometry = static_cast<int32_t>(
          (static_cast<uint32_t>(msg.state.left_wheel.lsb)) |
          (static_cast<uint32_t>(msg.state.left_wheel.low_byte) << 8) |
          (static_cast<uint32_t>(msg.state.left_wheel.high_byte) << 16) |
          (static_cast<uint32_t>(msg.state.left_wheel.msb) << 24));
      break;
    }
    case AgxMsgBmsDate: {
      const BMSDateMessage &msg = status_msg.body.bms_date_msg;
      state.SOC = msg.state.battery_SOC;
      state.SOH = msg.state.battery_SOH;
      state.bms_battery_voltage = static_cast<int16_t>(
            (static_cast<uint16_t>(msg.state.battery_voltage.low_byte)) |
            (static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8)) / 100.0;

      state.battery_current = static_cast<int16_t>(
            static_cast<uint16_t>(msg.state.battery_current.low_byte) |
            static_cast<uint16_t>(msg.state.battery_current.high_byte) << 8) / 10.0;
      //std::cout << state.bms_battery_voltage <<std::endl;

      state.battery_temperature = static_cast<int16_t>(
            (static_cast<uint16_t>(msg.state.battery_temperature.low_byte)) |
            (static_cast<uint16_t>(msg.state.battery_temperature.high_byte) << 8)) / 10.0;
      break;
    }
    case AgxMsgBmsStatus: {
      const BMSStatusMessage &msg = status_msg.body.bms_status_msg;
      state.Alarm_Status_1 = msg.state.Alarm_Status_1;
      state.Alarm_Status_2 = msg.state.Alarm_Status_2;
      state.Warning_Status_1 = msg.state.Warning_Status_1;
      state.Warning_Status_2 = msg.state.Warning_Status_2;
    }
  }
}
}  // namespace westonrobot
