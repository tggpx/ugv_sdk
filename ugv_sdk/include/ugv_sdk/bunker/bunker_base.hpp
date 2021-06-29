/*
 * bunker_base.hpp
 *
 * Created on: Jun 04, 2019 01:22
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef BUNKER_BASE_HPP
#define BUNKER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "ugv_sdk/mobile_base.hpp"
#include "ugv_sdk/proto/agx_msg_parser.h"
#include "ugv_sdk/bunker/bunker_types.hpp"

namespace westonrobot {
class BunkerBase : public MobileBase {
 public:
  BunkerBase(bool is_bunker_mini = false)
    : MobileBase(),is_bunker_mini_(is_bunker_mini){};
  ~BunkerBase() = default;

  // get robot state
  BunkerState GetBunkerState();

  void EnableCommandedMode();

  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel);

  // light control
  void SetLightCommand(const BunkerLightCmd &cmd);
  void DisableLightCmdControl();

 private:
  bool is_bunker_mini_ = false;
  // cmd/status update related variables
  std::mutex bunker_state_mutex_;
  std::mutex motion_cmd_mutex_;

  BunkerState bunker_state_;
  BunkerMotionCmd current_motion_cmd_;

  // internal functions
  void SendMotionCmd(uint8_t count);
  void SendLightCmd(const BunkerLightCmd &cmd, uint8_t count);


  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;

  void NewStatusMsgReceivedCallback(const AgxMessage &msg);
  static void UpdateBunkerState(const AgxMessage &status_msg, BunkerState &state);
};
}  // namespace westonrobot

#endif /* BUNKER_BASE_HPP */
