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

#include "analogio.h"
#include "constants.h"
#include "mainssync.h"
#include "simpleio.h"
#include "flowmeter.h"

#include <Arduino.h>
#include <avr/sleep.h>

static const uint8_t tempProbePins[] = {TEMP_PROBE_1_ADCPIN, TEMP_PROBE_2_ADCPIN, TEMP_PROBE_3_ADCPIN};

static bool g_lowRange = true;

// called when ADC interrupt occurs - we need an empty handler so the interrupt vector table can be populated
ISR(ADC_vect)
{
}

static int readAdcPin(uint8_t pin)
{
  // set the analog reference to external, select the channel and clear ADLAR
  ADMUX = ~_BV(REFS1) & ~_BV(REFS0) & ~_BV(ADLAR) & (pin & 0x0f);

  // the bandgap voltage takes some time to stabilise
  if (pin == BANDGAP_ADCPIN)
    delay(1);

  // enable ADC conversion complete interrupt and go to sleep - we'll wake up when there is data
  ADCSRA |= _BV(ADIE);
  set_sleep_mode(SLEEP_MODE_IDLE);

  // now sleep for a low noise measurement
  sleep_mode();
  // make sure the conversion is finished and we didn't wake up early for some reason
  while (ADCSRA & _BV(ADSC));

  ADCSRA &= ~_BV(ADIE);

  return ADC;
}

static double cleanReadAdcPin(uint8_t pin, int count = 2)
{
  // wait for mains zero-crossing so any mains interference is minimised
  MainsSyncManager::waitForMainsZeroCross();

  // read several times to get an average
  double readingFull = 0;
  readingFull += readAdcPin(pin);
  for (int i = 1; i < count; i++)
  {
    delay(1);
    readingFull += readAdcPin(pin);
  }

  return readingFull / count;
}

void AnalogIo::initialise()
{
  // make sure no voltage is being placed on Aref pin
  ADMUX &= ~_BV(REFS1) & ~_BV(REFS0);

  // disable the digital circuitry on the analogue input pins
  const uint8_t disableBits[] = {ADC0D, ADC1D, ADC2D, ADC3D, ADC4D, ADC5D};
  if (TEMP_PROBE_1_ADCPIN < 6)
    DIDR0 |= _BV(disableBits[TEMP_PROBE_1_ADCPIN]);
  digitalWrite(14 + TEMP_PROBE_1_ADCPIN, 0);

  if (TEMP_PROBE_2_ADCPIN < 6)
    DIDR0 |= _BV(disableBits[TEMP_PROBE_2_ADCPIN]);
  digitalWrite(14 + TEMP_PROBE_2_ADCPIN, 0);

  if (TEMP_PROBE_3_ADCPIN < 6)
    DIDR0 |= _BV(disableBits[TEMP_PROBE_3_ADCPIN]);
  digitalWrite(14 + TEMP_PROBE_3_ADCPIN, 0);

  if (LEVEL_METER_ADCPIN < 6)
    DIDR0 |= _BV(disableBits[LEVEL_METER_ADCPIN]);
  digitalWrite(14 + LEVEL_METER_ADCPIN, 0);

  // set up the temperature reading range
  pinMode(TEMP_LOW_RANGE_PIN, OUTPUT);
  digitalWrite(TEMP_LOW_RANGE_PIN, g_lowRange);

  // the first ADC result can be off because the reference voltage is being set to a new source, so do a read
  readAdcPin(BANDGAP_ADCPIN);
}

void AnalogIo::calibratePcbTemp()
{
  // get the present PCB temperature
  delay(1);
  while(Serial.available()) Serial.read();
  Serial.println(F("With the lid off, boiler cold, mains power on and AVR on-time > 5 minutes, what is the ambient temperature?"));
  while(!Serial.available());
  const double pcbTemp = Serial.parseFloat();

  const double newOffset = AnalogIo::getPcbTemperature() + PCBTEMP_OFFSET - pcbTemp;

  Serial.print(F("static const double AMBIENTTEMP_OFFSET = "));
  Serial.print(newOffset);
  Serial.println(F(";"));
}

static double getKnownResistance(int readingNum)
{
  // request low or high known resistance
  delay(1);
  while(Serial.available()) Serial.read();
  Serial.print(F("Using a nominal "));
  if (readingNum == 1) Serial.print(F("100"));
  if (readingNum == 2) Serial.print(F("120"));
  if (readingNum == 3) Serial.print(F("150"));
  Serial.println(F(" ohm resistor, what is the exact resistance?"));
  while(!Serial.available());

  return Serial.parseFloat();
}

static double getTempBasedResistance(int readingNum, AnalogIo::ETempProbe probe)
{
  double temperature;
  if (readingNum == 1)
  {
    // ensure tank full
    delay(1);
    while (Serial.available()) Serial.read();
    Serial.println(F("Ensure the water tank is full and then send a character."));
    while(!Serial.available());
    while(Serial.available()) Serial.read();

    // flush until temperature stops decreasing
    double minTemp = 1000.0;
    unsigned long minTempTime = micros();
    MainsSyncManager::setPumpPower(1.0);
    SimpleIo::switchSolenoid(true);
    while(true)
    {
      double temp = getTemperature(probe);
      if (temp < minTemp)
      {
        minTemp = temp;
        minTempTime = micros();
      }
      if (micros() - minTempTime > 10000000) break;
      delay(500);
    }
    MainsSyncManager::setPumpPower(0.0);
    SimpleIo::switchSolenoid(false);

    // request ambient temp
    delay(1);
    while(Serial.available()) Serial.read();
    Serial.println(F("Please enter the ambient/water temperature."));
    while(!Serial.available());
    temperature = Serial.parseFloat();
  }
  if (readingNum == 2)
  {
    // heat slowly until steam comes out (assume this happens at 100°C)
    double maxTemp = -1000.0;
    unsigned long maxTempTime = micros();
    MainsSyncManager::setHeaterPower(1000.0);
    SimpleIo::switchSolenoid(true);
    while(true)
    {
      double temp = getTemperature(probe);
      if (temp > maxTemp)
      {
        maxTemp = temp;
        maxTempTime = micros();
      }
      if (micros() - maxTempTime > 10000000) break;
      Serial.println(getTemperature(probe));
    }
    MainsSyncManager::setHeaterPower(0.0);
    SimpleIo::switchSolenoid(false);

    // we hope atmospheric pressure is close to 1013.25hPa
    temperature = 100.0;
  }

  // now return the resistance of a PT100 device at this temperature
  return PT100_A * temperature * temperature + PT100_B * temperature + PT100_C;
}

void AnalogIo::calibrateTempProbe()
{
  Serial.println(F("Calibrating temperature probe."));

  delay(1);
  while (Serial.available()) Serial.read();
  Serial.println(F("Which probe? (1/2/3)"));
  while (!Serial.available());
  ETempProbe probe = PROBE1 + Serial.parseInt() - 1;
  if (probe < PROBE1 || probe > PROBE3)
  {
    Serial.println(F("Bad probe number, exiting"));
    return;
  }

  delay(1);
  while (Serial.available()) Serial.read();
  Serial.println(F("What method would you like to use?"));
  Serial.println(F("1. Use known resistance values instead of PT100 probe."));
  Serial.println(F("2. Use known water temperatures with PT100 probe."));
  while (!Serial.available());
  char option = Serial.read();
  if (option != '1' && option != '2')
  {
    Serial.println(F("Bad option, exiting"));
    return;
  }

  // make sure we're in the lower temperature range
  if (SimpleIo::readSteamSwitch())
  {
    Serial.println(F("Switch off steam switch."));
    while(SimpleIo::readSteamSwitch());
  }

  // set up for known resistance 1
  double resistance1;
  if (option == '1') resistance1 = getKnownResistance(1);
  else resistance1 = getTempBasedResistance(1, probe);

  // get the ADC's reading
  double reading1 = cleanReadAdcPin(tempProbePins[probe]);

  // set up for known resistance 2
  double resistance2;
  if (option == '1') resistance2 = getKnownResistance(2);
  else resistance2 = getTempBasedResistance(2, probe);

  // get the ADC's reading
  double reading2 = cleanReadAdcPin(tempProbePins[probe]);

  // simulate steam mode by putting 0V on the steam swtich pin
  digitalWrite(TEMP_LOW_RANGE_PIN, false);

  // the third reading will be in range for a probe at 100 degrees but will need a nigher fixed resistor
  double resistance3;
  if (option == '1') resistance3 = getKnownResistance(3);
  else resistance3 = resistance2;

  double reading3 = cleanReadAdcPin(tempProbePins[probe]);

  digitalWrite(TEMP_LOW_RANGE_PIN, g_lowRange);

  // sanity check that the readings were in ADC range
  if (reading1 < 1.0 || reading1 > 1022.0)
  {
    Serial.print(F("Reading 1 was out of ADC range at "));
    Serial.println(reading1);
  }
  if (reading2 < 1.0 || reading2 > 1022.0)
  {
    Serial.print(F("Reading 2 was out of ADC range at "));
    Serial.println(reading2);
  }
  if (reading3 < 1.0 || reading3 > 1022.0)
  {
    Serial.print(F("Reading 3 was out of ADC range at "));
    Serial.println(reading3);
  }

  // output the results
  Serial.println(resistance1);
  Serial.println(reading1);
  Serial.println(resistance2);
  Serial.println(reading2);
  Serial.println(resistance3);
  Serial.println(reading3);
  Serial.print(F("static const double RESISTANCE_FACTOR = "));
  const double resistanceFactor = (resistance2 - resistance1) / (reading2 - reading1);
  Serial.print(resistanceFactor);
  Serial.println(F(";"));
  Serial.print(F("static const double RESISTANCE_OFFSET_LOWRANGE = "));
  Serial.print(resistance2 - reading2 * resistanceFactor);
  Serial.println(F(";"));
  Serial.print(F("static const double RESISTANCE_OFFSET_HIGHRANGE = "));
  Serial.print(resistance3 - reading3 * resistanceFactor);
  Serial.println(F(";"));
}

void AnalogIo::calibrateWaterLevelMeter()
{
  Serial.println(F("Calibrating water level meter."));

  // ensure tank full
  delay(1);
  while (Serial.available()) Serial.read();
  Serial.println(F("Ensure the water tank is empty and then send a character."));
  while(!Serial.available());

  // record ADC's reading
  double readingEmpty = cleanReadAdcPin(LEVEL_METER_ADCPIN);

  // ensure tank empty
  delay(1);
  while (Serial.available()) Serial.read();
  Serial.println(F("Ensure the water tank is full and then send a character."));
  while(!Serial.available());
  while(Serial.available()) Serial.read();

  // record ADC's reading
  double readingFull = cleanReadAdcPin(LEVEL_METER_ADCPIN);

  Serial.print(F("static const long LEVEL_PROBE_FULL_VALUE = "));
  Serial.print(readingFull);
  Serial.println(F(";"));
  Serial.print(F("static const long LEVEL_PROBE_EMPTY_VALUE = "));
  Serial.print(readingEmpty);
  Serial.println(F(";"));
}

static double getTemperaturePt100(double reading, bool lowRange, double factor, double offsetLowRange, double offsetHighRange)
{
  // extreme readings probably indicate wiring problems to the probe, return unknown temperature
  if (reading < 5.0 || reading > 1018.0)
    return NAN;

  const double resistance = factor * reading + (lowRange ? offsetLowRange : offsetHighRange);
  const double temperature = (-PT100_B + sqrt(PT100_B*PT100_B - 4.0 * PT100_A * (PT100_C - resistance))) / (2.0 * PT100_A);

  return temperature;
}

static double getTemperatureNtc(double reading, bool lowRange, double factor, double offsetLowRange, double offsetHighRange, double seriesR, double vcc, double r25, double beta)
{
  // extreme readings probably indicate wiring problems to the probe, return unknown temperature
  if (reading < 5.0 || reading > 1018.0)
    return NAN;

  double voltage = reading * factor + (lowRange ? offsetLowRange : offsetHighRange);
  const double resistance = seriesR * voltage / (vcc - voltage);
  const double temperature = 1.0 / (log(resistance / r25) / beta + 1.0 / (25 + 273.15)) - 273.15;

  return temperature;
}

static double getTemperatureKType(double reading, double factor, double offset)
{
  // extremely high probably indicate something is wrong, return unknown temperature
  // (low readings are normal when probe and AVR are at the same temperature - they
  //  could even be -ve, although the ADC can't measure that)
  if (reading > 1018.0)
    return NAN;

  // read the internal temperature of the AVR and assume the thermocouple probe terminal is at the same temperature
  const double referenceTemp = AnalogIo::getPcbTemperature();

  // something is really wrong if internal temperature is outside our table range for K Type probes so return an unknown temperatre
  static const int kTableLength = sizeof(KTYPE_TEMPS) / sizeof(KTYPE_TEMPS[0]);
  if (referenceTemp < KTYPE_TEMPS[0] || referenceTemp > KTYPE_TEMPS[kTableLength - 1])
    return NAN;

  // work out the K Type probe voltage at this temperature
  int i;
  for (i = 1; i < kTableLength && KTYPE_TEMPS[i] <= referenceTemp; i++);
  const double refTempMillivolts = KTYPE_VOLTAGES[i-1] + (KTYPE_VOLTAGES[i] - KTYPE_VOLTAGES[i-1]) / (KTYPE_TEMPS[i] - KTYPE_TEMPS[i-1]) * (referenceTemp - KTYPE_TEMPS[i-1]);

  // convert the probe's ADC reading to voltage and add it to the reference voltage
  const double kTypeMilliVolts = (reading * factor + offset) * 1000.0 + refTempMillivolts;

  // ensure the voltage is in the range of our K Type table
  if (kTypeMilliVolts < KTYPE_VOLTAGES[0] || kTypeMilliVolts > KTYPE_VOLTAGES[kTableLength - 1])
    return NAN;

  // finally use the K Type table to get the temperature of the probe
  for (i = 1; i < kTableLength && KTYPE_VOLTAGES[i] <= kTypeMilliVolts; i++);
  const double temperature = KTYPE_TEMPS[i-1] + (KTYPE_TEMPS[i] - KTYPE_TEMPS[i-1]) / (KTYPE_VOLTAGES[i] - KTYPE_VOLTAGES[i-1]) * (kTypeMilliVolts - KTYPE_VOLTAGES[i-1]);

  return temperature;
}

double AnalogIo::getPcbTemperature()
{
  // take a reading of internal 1.1V bandgap pin and the internal temperature probe
  const double bgVal = cleanReadAdcPin(BANDGAP_ADCPIN);
  const double tempVal = cleanReadAdcPin(TEMP_INTERNAL_ADCPIN, 8);

  // confirm there is current in the circuit
  // and AREF pin has a voltage of around 1.8V
  if (bgVal < 550 || bgVal > 700)
    // probe error so return an unknown temperature
    return NAN;

  // The datasheet says temperature should be measured with respect to the 1.1V reference which is
  // derived from the bandgap voltage. Experience also suggests that the AVR's internal temperature probe
  // voltage scales with badgap voltage. This is different for every device and is also dependent on VCC.
  const double bgVoltage = bgVal / 1024 * AREF_VOLTAGE;
  const double tempVoltage = tempVal / 1024 * AREF_VOLTAGE;
  const double tempVoltageAdjusted = tempVoltage * BANDGAP_VOLTAGE / bgVoltage;

  // Every device needs to be calibrated and we also need to offset the self-heating of the AVR to get to
  // amblient temperature.
  const double result = tempVoltageAdjusted * AVRTEMP_FACTOR - PCBTEMP_OFFSET;

  return result;
}

double AnalogIo::getTemperature(ETempProbe probe)
{
  // first take a reading of internal 1.1V bandgap pin to confirm there is current in the circuit
  // and AREF pin has a voltage of around 1.8V
  const int bgVal = readAdcPin(BANDGAP_ADCPIN);
  if (bgVal < 550 || bgVal > 700)
    // probe error so return an unknown temperature
    return NAN;

  // get an idea of the reading and if it is extreme, try the other measuring range
  double reading = readAdcPin(tempProbePins[probe]);
  if (reading < 5.0 || reading > 1018.0)
  {
    g_lowRange = !g_lowRange;
    digitalWrite(TEMP_LOW_RANGE_PIN, g_lowRange);
  }

  reading = cleanReadAdcPin(tempProbePins[probe]);

  switch(probe)
  {
#if defined PROBE1_PT100
    case PROBE1: return getTemperaturePt100(reading, g_lowRange, RESISTANCE_FACTOR1, RESISTANCE_OFFSET_LOWRANGE1, RESISTANCE_OFFSET_HIGHRANGE1);
#elif defined PROBE1_NTC
    case PROBE1: return getTemperatureNtc(reading, g_lowRange, VOLTAGE_FACTOR1, VOLTAGE_OFFSET_LOWRANGE1, VOLTAGE_OFFSET_HIGHRANGE1, SERIES_R1, VCC_VOLTAGE, R25_1, BETA1);
#elif defined PROBE1_KTYPE
    case PROBE1: return getTemperatureKType(reading, VOLTAGE_FACTOR1, VOLTAGE_OFFSET1);
#endif
#if defined PROBE2_PT100
    case PROBE2: return getTemperaturePt100(reading, g_lowRange, RESISTANCE_FACTOR2, RESISTANCE_OFFSET_LOWRANGE2, RESISTANCE_OFFSET_HIGHRANGE2);
#elif defined PROBE2_NTC
    case PROBE2: return getTemperatureNtc(reading, g_lowRange, VOLTAGE_FACTOR2, VOLTAGE_OFFSET_LOWRANGE2, VOLTAGE_OFFSET_HIGHRANGE2, SERIES_R2, VCC_VOLTAGE, R25_2, BETA2);
#elif defined PROBE2_KTYPE
    case PROBE2: return getTemperatureKType(reading, VOLTAGE_FACTOR2, VOLTAGE_OFFSET2);
#endif
#if defined PROBE3_PT100
    case PROBE3: return getTemperaturePt100(reading, g_lowRange, RESISTANCE_FACTOR3, RESISTANCE_OFFSET_LOWRANGE3, RESISTANCE_OFFSET_HIGHRANGE3);
#elif defined PROBE3_NTC
    case PROBE3: return getTemperatureNtc(reading, g_lowRange, VOLTAGE_FACTOR3, VOLTAGE_OFFSET_LOWRANGE3, VOLTAGE_OFFSET_HIGHRANGE3, SERIES_R3, VCC_VOLTAGE, R25_3, BETA3);
#elif defined PROBE3_KTYPE
    case PROBE3: return getTemperatureKType(reading, VOLTAGE_FACTOR3, VOLTAGE_OFFSET3);
#endif
  }

  // if we fall through to here, the specified probe does not exist, so return non-reading
  return NAN;
}

double AnalogIo::getAvgTemperature()
{
  return (getTemperature(PROBE1) + getTemperature(PROBE2)) / 2.0;
}

double AnalogIo::getWaterLevel()
{
  double avgReading = cleanReadAdcPin(LEVEL_METER_ADCPIN);
  double waterLevel = (avgReading - LEVEL_PROBE_EMPTY_VALUE) / (LEVEL_PROBE_FULL_VALUE - LEVEL_PROBE_EMPTY_VALUE);
  waterLevel = max(0.0, min(1.0, waterLevel)) * RESERVOIR_VOLUME;

  return waterLevel;
}
