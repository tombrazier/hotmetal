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
#include "mainssync.h"
#include "simpleio.h"

#include <Arduino.h>

static volatile unsigned long g_halfPulseCount = 0;;
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
# measured data - pulse rates in pulse/s and flow rates in mL/s
pulseRates = (5.13, 6.03, 7.00, 8.40, 9.52, 10.93, 12.75, 14.63, 16.10, )
flowRates = (0.95, 1.55, 2.05, 2.65, 3.55, 4.70, 6.10, 7.35, 8.60, )
# from the known ratio of pulse count to volume at given flow rates, build the related array of pulse rates
pulsesPerMl = tuple([x / y for x, y in zip(pulseRates, flowRates)])
# interpolate (it turns out that interpolating pulses per ml gives a much better curve than interpolating mls per pulse - so we'll invert later)
func = scipy.interpolate.interp1d(pulseRates, pulsesPerMl, kind='quadratic')
xarray = [pulseRates[0] + (pulseRates[-1] - pulseRates[0]) / 20 * x for x in range(21)]
yarray = func(xarray)
print('static const double pulseRates[] = {%s};' % ', '.join(['%.3f' % x for x in xarray]))
print('static const double mlPerPulse[] = {%s};' % ', '.join(['%.3f' % (1.0/y) for y in yarray]))
*/

  static const double pulseRates[] = {5.130, 5.678, 6.227, 6.776, 7.324, 7.873, 8.421, 8.970, 9.518, 10.067, 10.615, 11.164, 11.712, 12.261, 12.809, 13.358, 13.906, 14.455, 15.003, 15.552, 16.100};
  static const double mlPerPulse[] = {0.185, 0.230, 0.269, 0.288, 0.298, 0.304, 0.316, 0.341, 0.373, 0.399, 0.419, 0.438, 0.454, 0.468, 0.480, 0.488, 0.493, 0.500, 0.509, 0.520, 0.534};
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
  g_halfPulseCount++;

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
  delay(1);
  while (Serial.available()) Serial.read();
  Serial.println(F("Fill reservoir, place a measuring cup under steam wand and crack steam valve open (so flow is fairly constant, not pulsing)."));
  while(!Serial.available())
  while(Serial.available()) Serial.read();

  // run the pump at varying power leves for 20s and measure the number of pulses and then ask what volume was pumped
  double pulseRates[10];
  double flowRates[10];
  double power = 0.07508468627929688; // i.e. 0.75 ^ 10
  for (int i = 0; i < 10; i++)
  {
    power /= 0.75;

    uint8_t oldSREG = SREG;
    cli();
    unsigned long halfPulseCount = g_halfPulseCount;
    SREG = oldSREG;
    uint32_t startTime = millis();
    MainsSyncManager::setPumpPower(power);
    while (millis() - startTime < 20000);
    MainsSyncManager::setPumpPower(0.0);

    Serial.println(F("Measure the volume, replace the cup empty and enter the volume."));
    while(Serial.available()) Serial.read();
    while(!Serial.available());
    double volume = Serial.parseFloat();

    oldSREG = SREG;
    cli();
    halfPulseCount = g_halfPulseCount - halfPulseCount;
    SREG = oldSREG;

    pulseRates[i] = halfPulseCount / 2.0 / 20.0;
    flowRates[i] = volume / 20.0;
    Serial.print(pulseRates[i]);
    Serial.print(", ");
    Serial.println(flowRates[i]);
  }
  Serial.print("pulseRates = (");
  for (int i = 0; i < 10; i++) { Serial.print(pulseRates[i]); Serial.print(", "); }
  Serial.println(")");
  Serial.print("flowRates = (");
  for (int i = 0; i < 10; i++) { Serial.print(flowRates[i]); Serial.print(", "); }
  Serial.println(")");

  while(Serial.available()) Serial.read();
  Serial.println(F("Close the steam valve and send a character to continue."));
  while(!Serial.available())
  while(Serial.available()) Serial.read();
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

unsigned long FlowMeter::getPulseCount()
{
  uint8_t oldSREG = SREG;
  cli();
  unsigned long res = g_halfPulseCount / 2;
  SREG = oldSREG;
  return res;
}
