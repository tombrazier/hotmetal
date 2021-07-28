/*
 *     Copyright (C) 2010-2020 Tom Brazier (http://tomblog.firstsolo.net)
 *     
 *     This file is part of Hot Metal Brewbot.
 *     
 *     Hot Metal Brewbot is free software: you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation, either version 3 of the
 *     License, or (at your option) any later version.
 *     
 *     Hot Metal Brewbot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *     
 *     You should have received a copy of the GNU General Public License
 *     along with Hot Metal Brewbot .  If not, see <https://www.gnu.org/licenses/>.
 */

#include "simpleio.h"
#include "constants.h"

#include <Arduino.h>

void SimpleIo::initialise()
{
  // set up steam and pump switch pins as inputs and pull them
  // high, the switches will pull them low
  pinMode(STEAM_SW_PIN, INPUT_PULLUP);
  pinMode(PUMP_SW_PIN, INPUT_PULLUP);

  // set up temp okay LED output
  digitalWrite(TEMP_OKAY_LED_PIN, false);
  pinMode(TEMP_OKAY_LED_PIN, OUTPUT);

  // set the solenoid up as a simple on/off output, with no
  // power initially
  digitalWrite(SOLENOID_PIN, false);
  pinMode(SOLENOID_PIN, OUTPUT);

  // set the buzzer up as a simple on/off output, with no
  // power initially
  digitalWrite(PIEZO_BUZZER_PIN, false);
  pinMode(PIEZO_BUZZER_PIN, OUTPUT);
}

bool SimpleIo::readSteamSwitch()
{
  return !digitalRead(STEAM_SW_PIN);
}

bool SimpleIo::readPumpSwitch()
{
  return !digitalRead(PUMP_SW_PIN);
}

void SimpleIo::setTempOkayLed(bool on)
{
  digitalWrite(TEMP_OKAY_LED_PIN, on);
}

void SimpleIo::switchBuzzer(bool on)
{
  digitalWrite(PIEZO_BUZZER_PIN, on);
}

void SimpleIo::switchSolenoid(bool on)
{
  digitalWrite(SOLENOID_PIN, on);
}

bool SimpleIo::solenoidOn()
{
  uint8_t bit = digitalPinToBitMask(SOLENOID_PIN);
  uint8_t port = digitalPinToPort(SOLENOID_PIN);
  volatile uint8_t *out = portOutputRegister(port);
  return !!(*out & bit);
}
