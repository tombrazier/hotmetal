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

#include "mainssync.h"
#include "constants.h"

#include <Arduino.h>

static volatile double g_mainsCycleLength = MAINS_CYCLE_LENGTH;
static volatile double g_pumpPower = 0.0;
static volatile uint16_t g_pumpDutyCycle = 0;
static volatile int16_t g_energyAvailPerHalfCycle = 0.0;  // integer value representing real number * 256
static volatile int16_t g_energyReqdPerHalfCycle = 0.0;   // integer value representing real number * 256

static inline uint16_t encodeReal(double val) {return val * 256;}
static inline uint16_t roundReal(uint16_t val) {return (val + (1<<7)) >> 8;}

// invoked at mains voltage zero-crossing
ISR(PCINT2_vect)
{
  bool polarity = digitalRead(MAINS_POLARITY_PIN);

  // eliminate very fast noise that changes the input twice before the ISR can respond
  static bool expected = true;
  if (polarity != expected)
    return;
  expected = !expected;

  // if this is a zero-crossover heading towards positive (i.e. if pin is low) then the signal
  // edge from the mains crossing circuit is sharper, so use this as our zero-crossing reference
  if (!polarity)
  {
    // if the time since last trigger is more than 20% off from expected mains mains cycle length, reject this as noise
    static uint32_t lastTime = 0;
    uint32_t now = micros();;
    int32_t microsSinceLastTime = now - lastTime;
    lastTime = now;
    if (abs(microsSinceLastTime - int32_t(MAINS_CYCLE_LENGTH * 1000000)) < uint32_t(MAINS_CYCLE_LENGTH * 1000000 * 0.2))
    {
      // make sure TCNT2 flips over to zero at least 1 tick before mains zero-crossing
      static const uint8_t expectedTcnt2 = 2 + (MAINS_CROSS_LATENCY * F_CPU / 1024);
      TCNT2 = expectedTcnt2;

      // update our record of the mains cycle length, keeping a running average
      g_mainsCycleLength = 0.8 * g_mainsCycleLength + 0.2e-6 * microsSinceLastTime;

      // and update the range of TCNT2
      const double ocr2a = g_mainsCycleLength / 2 * (F_CPU / 1024) - 1;
      OCR2A = ocr2a + 0.5;
    }
  }
}

// once the AC phase measurement is stabilised, timer2 overflows just about at zero crossover
ISR(TIMER2_OVF_vect)
{
  // Keep track of two numbers:
  //   First: the energy that has been delivered minus the energy that has been requested.
  //   Second: the energy that has been delivered in this half-cycle minus that delivered in the other.
  static int16_t energyError = 0;     // +ve means more energy has been delivered than requested
  static int16_t cycleDelta = 0;      // +ve means this half-cycle has delivered the most energy

  // if there is an energy deficit and this cycle has generated less energy than the other cycle
  // then switch on for this cycle
  if (g_energyReqdPerHalfCycle > 0 && energyError <= 0 && cycleDelta <= 0)
  {
    digitalWrite(BOILER_POWER_PIN, true);
    energyError += g_energyAvailPerHalfCycle;
    cycleDelta += g_energyAvailPerHalfCycle;
  }
  else
    digitalWrite(BOILER_POWER_PIN, false);

  // reduce the total error by what should have been delivered
  energyError -= g_energyReqdPerHalfCycle;

  // flip the value of cycleDelta because the next time this function is called will be for the other half-cycle
  cycleDelta = -cycleDelta;

  // only change pump state every second half-cycle
  static bool thisHalfCycle = true;
  if (thisHalfCycle)
  {
    static int16_t excessCyclesPumped = 0;
    if (g_pumpDutyCycle > 0 && excessCyclesPumped <= 0)
    {
      digitalWrite(PUMP_POWER_PIN, true);
      excessCyclesPumped += encodeReal(1.0);
    }
    else
      digitalWrite(PUMP_POWER_PIN, false);

    excessCyclesPumped -= g_pumpDutyCycle;
  }
  thisHalfCycle = !thisHalfCycle;
}

void MainsSyncManager::initialise()
{
  // set up heater output pin and switch heater off initially
  digitalWrite(BOILER_POWER_PIN, false);
  pinMode(BOILER_POWER_PIN, OUTPUT);

  // set up pump output pin and switch pump off initially
  digitalWrite(PUMP_POWER_PIN, false);
  pinMode(PUMP_POWER_PIN, OUTPUT);

  // set up mains cycle sense pin
  pinMode(MAINS_POLARITY_PIN, INPUT_PULLUP);

  // set up timer/counter2's prescaler to 1024 and mode to Fast PWM with TOP=OCR2A
  // now we get an 8 bit counter which can count up as far as 16.384ms or 32.768ms
  // OCR2A will determine the max (which we'll line up with the mains AC half-cycle)
  OCR2A = OCR2B = 0xff;
  TCCR2B |= _BV(CS20);
  TCCR2B |= _BV(CS21);
  TCCR2B |= _BV(CS22);
  TCCR2A |= _BV(WGM20);
  TCCR2A |= _BV(WGM21);
  TCCR2B |= _BV(WGM22);
  TIMSK2 |= _BV(TOIE2);
  pinMode(PUMP_POWER_PIN, OUTPUT);

  // The initial timer/counter2 mode is phase correct PWM with TOP = 0xff. Experimentation shows
  // we have to let this complete its present cycle before the mode will change. If we don't
  // wait for this, the zero-cross interrupt can continually reset TCNT2 which then never reaches
  // 0xff and so the correct mode never gets set.
  while(TCNT2 == 0x00);
  while(TCNT2 != 0x00);

  // set up mains sense pin to generate a PCINT2 when switching on or off
  PCMSK2 = 0x00;
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(MAINS_POLARITY_PCINT);
}

bool MainsSyncManager::ready()
{
  // after two mains cycles we should be in sync
  return micros() >= uint32_t(2000000 * MAINS_CYCLE_LENGTH);
}

double MainsSyncManager::getMainsCycleLength()
{
  uint8_t oldSREG = SREG;
  cli();
  double res = g_mainsCycleLength;
  SREG = oldSREG;
  return res;
}

void MainsSyncManager::setPumpPower(double power)
{
  // clip value
  g_pumpPower = min(1.0, max(0.0, power));

  uint8_t oldSREG = SREG;
  cli();

  g_pumpDutyCycle = encodeReal(g_pumpPower);

  SREG = oldSREG;
}

double MainsSyncManager::getPumpPower()
{
  // note: no need to protect with cli() because this value is never set inside interrupts
  return g_pumpPower;
}

void MainsSyncManager::setHeaterPower(double power)
{
  uint8_t oldSREG = SREG;

  // clip value in watts to range 0.0 to SAFE_HEATER_POWER
  power = min(SAFE_HEATER_POWER, max(0.0, power));

  cli();
  double mainsCycleLength = g_mainsCycleLength;
  SREG = oldSREG;

  // calculate how the energy error accrues during off and on cycles
  double halfCycleTime = mainsCycleLength / 2.0;
  double energyAvailPerHalfCycle = MAX_HEATER_POWER * halfCycleTime;
  double energyReqdPerHalfCycle = power * halfCycleTime;

  cli();
  g_energyAvailPerHalfCycle = encodeReal(energyAvailPerHalfCycle);
  g_energyReqdPerHalfCycle = encodeReal(energyReqdPerHalfCycle);
  SREG = oldSREG;
}

void MainsSyncManager::waitForMainsZeroCross()
{
  // wait till TCNT2 == 0, which is mains zero cross - and for safety, just in case we miss zero,
  // also test for TCNT2's value decreasing
  for(uint8_t tcnt2 = TCNT2; TCNT2 > 0 && TCNT2 >= tcnt2; tcnt2 = TCNT2);
}
