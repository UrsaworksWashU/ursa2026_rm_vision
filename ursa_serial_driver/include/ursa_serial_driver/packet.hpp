// Copyright 2025 Ursa Works
// Licensed under the Apache-2.0 License.
//
// Wire protocol between Jetson (upper) and C-board STM32 (lower) via USB-CDC VCP.
// Packet sizes and field layout must stay in sync with Infantry_Bottom
// modules/master_machine/master_process.c (VISION_USE_VCP section).

#ifndef URSA_SERIAL_DRIVER__PACKET_HPP_
#define URSA_SERIAL_DRIVER__PACKET_HPP_

#include <cstdint>

namespace ursa_serial_driver
{

// C-board -> Jetson, 34 bytes, header 0x5A
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0=red enemy  1=blue enemy
  uint8_t task_mode : 2;     // 0=auto  1=aim  2=buff
  uint8_t reset_tracker : 1;
  uint8_t is_play : 1;
  uint8_t change_target : 1;
  uint8_t reserved : 2;
  float roll;        // rad
  float pitch;       // rad
  float yaw;         // rad
  float aim_x;       // unused (filled 0)
  float aim_y;       // unused (filled 0)
  float aim_z;       // unused (filled 0)
  uint16_t game_time;   // unused (filled 0)
  uint32_t timestamp;   // ms, HAL_GetTick()
  uint16_t checksum;
} __attribute__((packed));

// Jetson -> C-board, 12 bytes, header 0xA5
struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t mode;    // 0=no ctrl  1=aim no fire  2=aim+fire
  float yaw;       // rad, absolute gimbal target
  float pitch;     // rad, absolute gimbal target
  uint16_t checksum;
} __attribute__((packed));

static_assert(sizeof(ReceivePacket) == 34, "ReceivePacket must be 34 bytes");
static_assert(sizeof(SendPacket) == 12, "SendPacket must be 12 bytes");

}  // namespace ursa_serial_driver

#endif  // URSA_SERIAL_DRIVER__PACKET_HPP_
