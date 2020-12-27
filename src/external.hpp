//
// Created by Stercutius on 27/12/2020
// Des constantes doivent être partagés entre master et slaves pour que la communication
// s'établisse correctement. Le code source des slaves a accès à ce fichier afin
// que le master puisse les trouver sur le bus I²C et que les slaves puissent interpréter
// les ordres qu'ils recoivent.
//

#pragma once

namespace external {

constexpr uint8_t hammers_id = 1;
constexpr uint8_t pumps_id = 2;

constexpr uint8_t set_hammer_angle_id = 0;
constexpr uint8_t raise_hammer_id = 1;
constexpr uint8_t lower_hammer_id = 2;

constexpr uint8_t toggle_valve_id = 0;
constexpr uint8_t suck_id = 0;

} // namespace external
