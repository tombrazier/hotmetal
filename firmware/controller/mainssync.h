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

#ifndef COFFEEID_MAINSSYNCMANAGER_H
#define COFFEEID_MAINSSYNCMANAGER_H

/*
 * Public interface to all the code related to timer/counter2 which is kept
 * in sync with the AC mains cycle and which manages the power output to both
 * the pump and the heater. Both of these components are driven by mains AC
 * and must be synced to the AC cycle.
 * The code itself is encapsulated in the .cpp file.
 */

namespace MainsSyncManager
{
  // set up the related IO hardware, interrupts, etc.
  void initialise();

  // returns true once the zero-crossing sense system has stablised
  bool ready();

  // for diagnostics, return the length of the mains AC half-cycle
  double getMainsCycleLength();

  // set and get pump power on a scale of 0.0 to 1.0
  void setPumpPower(double power);
  double getPumpPower();

  // set heater element power in watts
  void setHeaterPower(double power);

  // wait until the zero crossing has just happened - good for ADC noise elimination
  void waitForMainsZeroCross();
}

#endif
