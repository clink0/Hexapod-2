/*
 * MagCal.h — Save / load ICM-20948 magnetometer calibration via EEPROM.
 *
 * Copy this file into any sketch folder that needs magnetometer calibration.
 *
 * Usage:
 *   #include "MagCal.h"
 *
 *   MagCal cal;
 *   if (!magCalLoad(cal)) { ... run calibration routine ... }
 *   magCalSave(cal);
 *
 *   float mx = (rawX - cal.offsetX) * cal.scaleX;
 *   float my = (rawY - cal.offsetY) * cal.scaleY;
 */

#pragma once
#include <EEPROM.h>

#define MAGCAL_EEPROM_ADDR 0       // Starting byte address in EEPROM
#define MAGCAL_MAGIC       0xCAFE  // Sentinel to detect valid saved data

struct MagCal {
  uint16_t magic;
  float offsetX;
  float offsetY;
  float scaleX;
  float scaleY;
};

// Returns true if valid calibration was found in EEPROM and loaded into `cal`.
inline bool magCalLoad(MagCal &cal) {
  EEPROM.get(MAGCAL_EEPROM_ADDR, cal);
  return cal.magic == MAGCAL_MAGIC;
}

// Writes calibration to EEPROM.
inline void magCalSave(MagCal &cal) {
  cal.magic = MAGCAL_MAGIC;
  EEPROM.put(MAGCAL_EEPROM_ADDR, cal);
}

// Prints current calibration values to Serial.
inline void magCalPrint(const MagCal &cal) {
  Serial.print("Offset  X="); Serial.print(cal.offsetX);
  Serial.print("  Y=");        Serial.println(cal.offsetY);
  Serial.print("Scale   X="); Serial.print(cal.scaleX);
  Serial.print("  Y=");        Serial.println(cal.scaleY);
}
