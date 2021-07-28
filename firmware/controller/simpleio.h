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

#ifndef COFFEEID_SIMPLEIO_H
#define COFFEEID_SIMPLEIO_H

/*
 * Public interface to all the code related to the simple on/off
 * IO hardware.
 * The code itself is encapsulated in the .cpp file.
 */

namespace SimpleIo
{
  // set up the related IO hardware, interrupts, etc.
  void initialise();

  // read the switches on the front panel
  bool readSteamSwitch();
  bool readPumpSwitch();

  // set status LEDs
  void setTempOkayLed(bool on);

  // set the buzzer's state
  void switchBuzzer(bool on);

  // set/get the solenoid activation state
  void switchSolenoid(bool on);
  bool solenoidOn();
}

#endif
