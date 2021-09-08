/*
 * agilex_types.h
 *
 * Created on: Jul 09, 2021 21:57
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef AGILEX_TYPES_H
#define AGILEX_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  LIGHT_DISABLE = 0x00, // �ر�
  LIGHT_GO_FORWARD, // ǰ��
  LIGHT_GO_BACK, // ����
  LIGHT_TURN_LEFT, // ��ת
  LIGHT_TURN_RIGHT, // ��ת
  LIGHT_TURN_AROUND, // ԭ����ת
  LIGHT_FAULT, // ����
  LIGHT_IDLE,  // ����
  LIGHT_WORK // ��ҵ
} LightMode;

typedef enum {
  VehicleStateNormal = 0x00,
  VehicleStateEStop = 0x01,
  VehicleStateException = 0x02
} VehicleState;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  CONTROL_MODE_RC = 0x00,
  CONTROL_MODE_CAN = 0x01,
  CONTROL_MODE_UART = 0x02
} ControlMode;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  BRAKE_MODE_UNLOCK = 0x00,
  BRAKE_MODE_LOCK = 0x01
} BrakeMode;

typedef enum {
  RC_SWITCH_UP = 0,
  RC_SWITCH_MIDDLE,
  RC_SWITCH_DOWN
} RcSwitchState;

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_TYPES_H */
