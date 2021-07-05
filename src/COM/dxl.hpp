#pragma once

#include <SoftwareSerial.h>

#include "Config/PinMapping.h"

extern SoftwareSerial hammer_dxl_stream;
extern SoftwareSerial arm_dxl_stream;

constexpr uint8_t hammer_dxl_ids[] { 1, 2 };
constexpr uint32_t hammer_dxl_max_offset = 500;
constexpr uint32_t hammer_dxl_home_positions[] { 4096 / 4, 4096 / 4 };
constexpr uint32_t hammer_dxl_raised_offsets[] { hammer_dxl_max_offset, -hammer_dxl_max_offset };
constexpr uint32_t hammer_dxl_lowered_offsets[] { -hammer_dxl_max_offset , hammer_dxl_max_offset };

constexpr uint8_t arm_dxl_ids[] { 1, 2 };
constexpr uint32_t arm_dxl_lowered_positions[] { 0, 4096 / 4 };
constexpr uint32_t arm_dxl_raised_positions[] { 4096 / 4, 0 };

constexpr unsigned int dxl_interframe_delay_us = 5000;
