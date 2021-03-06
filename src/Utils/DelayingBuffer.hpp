//
// Created by Stercutius on 04/03/2021.
// Ce fichier procure un buffer global qui permet de stocker des informations à envoyer.
// L'utilité de ce buffer est de servir de rustine lorsqu'il faut envoyer beaucoup d'information et que ça ralenti le
// reste du programme (à l'heure où j'écris ses lignes, lire les valeurs des codeuses et les envoyer à l'ordinateur à
// une fréquence de l'ordre du kHz est impossible parce que l'envoi de donnée via Serial est couteux en terme de cycle
// machine -- utilisez le code 'rotary_encoder' dans le repo 'testing' pour vous en rendre compte).
// C'est très moche, mais c'est le seul truc qui semble marcher avec le design du code en place donc bon...
//
// String ne permet pas de connaître la capacité totale réservée avec la méthode 'String::reserve'. A la place, vous
// pouvez utiliser la globale 'capacity' pour garder capacité de 'buffer'.
//

#pragma once

#include <Arduino.h>

namespace dbuf {

extern String buffer;
extern size_t capacity;

} // namespace dbuf
