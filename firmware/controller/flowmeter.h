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

#ifndef COFFEEID_FLOWMETER_H
#define COFFEEID_FLOWMETER_H

/*
 * Public interface to all the code related to the flow meter.
 * The code itself is encapsulated in the .cpp file.
 */

namespace FlowMeter
{
  // set up the related IO hardware, interrupts, etc.
  void initialise();

  // run calibration routines
  void calibrate();

  // returns the present flow rate in ml/s
  double getFlowRate();

  // returns the total volume of water measured in ml
  double getTotalVolume();

  // get the tota number of full pulses on the flow meter
  unsigned long getPulseCount();
}

#endif
