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

#include "flowmeter.h"
#include "constants.h"

#include <Arduino.h>

static volatile unsigned long g_lastChangeTime = 0;
static volatile unsigned long g_lastFlowPulseTime = 0;
static volatile double g_flowHalfPeriods[2] = {1.0e6, 1.0e6};
static volatile double g_totalVolume = 0.0;

static double getMlsPerPulse(double pulseRate)
{
  // The flow meter is not linear at low flow rates.  This function returns the ratio of mls
  // to pulses at the given pulse rate.  If the pulse rate is too low, then a value of NaN is
  // returned.

  // The following python code generates the lookup table from experimentally determined
  // relationships between flow and numbers of pulses per ml.  Above the given data, the
  // curve is linear with a factor of 1.925ish pulses per ml.  Below the given data, the
  // pulses cease, so the flow cannot be determined.
/*
import scipy.interpolate
# measured data
pulsesPerMl=(0.74, 1.22, 1.35, 1.55, 1.7, 1.84, 1.925, 1.925)
flowRates=(1.02, 1.14, 1.22, 1.38, 1.61, 2.01, 2.6, 3.0)
# from the known ratio of pulse count to volume at given flow rates, build the related array of pulse rates
pulseRates = tuple([x * y for x, y in zip(pulsesPerMl, flowRates)])
# interpolate (it turns out that interpolating pulses per ml gives a much better curve than interpolating mls per pulse - so we'll invert later)
func = scipy.interpolate.interp1d(pulseRates, pulsesPerMl, kind='quadratic')
xarray = [pulseRates[0] + (pulseRates[-1] - pulseRates[0]) / 20 * x for x in range(21)]
yarray = func(xarray)
print 'static const double pulseRates[] = {%s};' % ', '.join(['%.3f' % x for x in xarray])
print 'static const double mlPerPulse[] = {%s};' % ', '.join(['%.3f' % (1.0/y) for y in yarray])
*/

  static const double pulseRates[] = {0.755, 1.006, 1.257, 1.508, 1.759, 2.010, 2.261, 2.512, 2.763, 3.014, 3.265, 3.516, 3.767, 4.018, 4.269, 4.520, 4.771, 5.022, 5.273, 5.524, 5.775};
  static const double mlPerPulse[] = {0.884, 0.688, 0.591, 0.539, 0.506, 0.484, 0.470, 0.464, 0.466, 0.469, 0.473, 0.477, 0.477, 0.472, 0.467, 0.466, 0.466, 0.469, 0.471, 0.473, 0.475};
  static const int pulseTableLength = sizeof(pulseRates) / sizeof(pulseRates[0]);
  static const double interval = (pulseRates[pulseTableLength - 1] - pulseRates[0]) / (pulseTableLength - 1);

  // interpolate to get flowrate
  int tablePos = (pulseRate - pulseRates[0]) / interval;
  if (tablePos < 0)
    return NAN;
  if (tablePos >= pulseTableLength - 1)
    return mlPerPulse[pulseTableLength - 1];
  return mlPerPulse[tablePos] + (mlPerPulse[tablePos + 1] - mlPerPulse[tablePos]) * (pulseRate - pulseRates[tablePos]) / interval;
}

// PCINT1 is connected to the water flow meter - and the interrupt is called whenever state changes
ISR(PCINT1_vect)
{
  // NOTE: there is a very small risk that there might be just over a multiple of 70ish minutes between
  // pulses, giving and artificial high flow rate due to overflow in micros().  I'm happy to accept the
  // problem because I don't anticipate running the beast for more than an hour.

  // record time now and debounce input to filter out errors caused by pump vibration
  unsigned long now = micros();
  if ((now - g_lastChangeTime) < FLOW_METER_DEBOUNCE_TIME * 1000000.0)
  {
    g_lastChangeTime = now;
    return;
  }
  g_lastChangeTime = now;

  // get the length of the last half-cycle
  double timeSinceLastTick = 1.0 / 1000000.0 * (now - g_lastFlowPulseTime);
  g_lastFlowPulseTime = now;

  // record the length of both the last half-pulses (i.e. one whole cycle)
  g_flowHalfPeriods[0] = g_flowHalfPeriods[1];
  g_flowHalfPeriods[1] = timeSinceLastTick;

  // update the total volume which has been pumped through the system
  double mlsPerPulse = getMlsPerPulse(1.0 / (g_flowHalfPeriods[0] + g_flowHalfPeriods[1]));
  if (!isnan(mlsPerPulse))
  {
    g_totalVolume += mlsPerPulse / 2.0;
    return;
  }

  // if we get here, then flow must recently have started - try doubling the last half cycle's time
  // if that doesn't work, then we'll lose track of a very small quantity and that is okay
  mlsPerPulse = getMlsPerPulse(0.5 / g_flowHalfPeriods[1]);
  if (!isnan(mlsPerPulse))
    g_totalVolume += mlsPerPulse / 2.0;
}

void FlowMeter::initialise()
{
  // set up flow meter pulse-signal pin - bias the pin high because meter supposedly is open collector
  pinMode(FLOW_METER_PIN, INPUT_PULLUP);

  // set up the flow meter to generate a PCINT1 when switching on or off
  PCMSK1 = 0x00;
  PCICR |= _BV(PCIE1);
  PCMSK1 |= _BV(FLOW_METER_PCINT);
}

void FlowMeter::calibrate()
{
  Serial.println(F("Calibrating flow meter..."));
}

double FlowMeter::getFlowRate()
{
  // This function returns the flow rate as long as it is high enough to measure - otherwise it returns zero.
  // Given the usages of the function (controlling flow and testing for a significant flow), returning zero
  // is just as good as returning the actual value.

  // work out rate at which we're getting full pulses (from high to high or low to low - we don't care)
  // if the flow is slowing, give a better result by using the time since the last tick instead of the 0th
  // value (the one in the same half-cycle as we're in now)
  uint8_t oldSREG = SREG;
  cli();
  unsigned long now = micros();
  double timeSinceLastTick = 1.0 / 1000000.0 * (now - g_lastFlowPulseTime);
  double pulseRateFull = 1.0 / (max(g_flowHalfPeriods[0], timeSinceLastTick) + g_flowHalfPeriods[1]);
  double pulseRatePart = 0.5 / max(g_flowHalfPeriods[1], timeSinceLastTick);
  SREG = oldSREG;

  // calculate the flow rate and return it
  double mlsPerPulse = getMlsPerPulse(pulseRateFull);
  if (!isnan(mlsPerPulse))
    return mlsPerPulse * pulseRateFull;

  // if we got here, the pulse rate was too low to calculate a flow rate, so we'll try the less reliable
  // information from the most recent half-cycle - if this works, then there have been two pulses
  // since the flow started up after being stopped and we'll briefly return a less accurate number which
  // is nonetheless the very best we know how to manage
  mlsPerPulse = getMlsPerPulse(pulseRatePart);
  if (!isnan(mlsPerPulse))
    return mlsPerPulse * pulseRatePart;

  // if we got here, then the best, most recent information we have indicates a low flow rate - the best
  // we can do is say that flow has stopped
  return 0.0;
}

double FlowMeter::getTotalVolume()
{
  uint8_t oldSREG = SREG;
  cli();
  double res = g_totalVolume;
  SREG = oldSREG;
  return res;
}
