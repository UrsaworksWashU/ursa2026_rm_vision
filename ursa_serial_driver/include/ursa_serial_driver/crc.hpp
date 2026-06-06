// Copyright 2022 ChenJun — ported from rm_serial_driver unchanged.
// Licensed under the Apache-2.0 License.
//
// CRC16-IBM-SDLC, poly 0x8408, init 0xFFFF.
// Table is identical to SP_CRC16_TABLE in Infantry_Bottom master_process.c.

#ifndef URSA_SERIAL_DRIVER__CRC_HPP_
#define URSA_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>

namespace crc16
{

uint16_t Get_CRC16_Check_Sum(const uint8_t * msg, uint32_t len, uint16_t init);
uint32_t Verify_CRC16_Check_Sum(const uint8_t * msg, uint32_t len);
void     Append_CRC16_Check_Sum(uint8_t * msg, uint32_t len);

}  // namespace crc16

#endif  // URSA_SERIAL_DRIVER__CRC_HPP_
