#pragma once

#include <SoftwareSerial.h>

#include "Config/PinMapping.h"

extern SoftwareSerial hammer_dxl_stream;
constexpr uint8_t hammer_dxl_ids[] { 1, 2 };
constexpr uint32_t hammer_dxl_max_offset = 500;
constexpr uint32_t hammer_dxl_home_positions[] { 4095 / 4, 4095 / 4 };
constexpr uint32_t hammer_dxl_raised_offsets[] { hammer_dxl_max_offset, -hammer_dxl_max_offset };
constexpr uint32_t hammer_dxl_lowered_offsets[] { -hammer_dxl_max_offset , hammer_dxl_max_offset };
constexpr unsigned int dxl_interframe_delay_us = 5000;
