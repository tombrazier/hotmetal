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

#ifndef COFFEEID_ANALOGIO_H
#define COFFEEID_ANALOGIO_H

/*
 * Public interface to all the code related to the temperature probe.
 * The code itself is encapsulated in the .cpp file.
 */

namespace AnalogIo
{
    typedef enum {PROBE1, PROBE2, PROBE3} ETempProbe;

  // set up the related IO hardware, interrupts, etc.
  void initialise();

  // run calibration routines
  void calibratePcbTemp();
  void calibrateTempProbe();
  void calibrateWaterLevelMeter();

  // read the boiler temperature in °C from the probes
  double getPcbTemperature();
  double getTemperature(ETempProbe probe);
  double getAvgTemperature();

  // returns the present reservoir volume in ml
  double getWaterLevel();
}

#endif
