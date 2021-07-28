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

#include "calibration.h"
#include "constants.h"
#include "mainssync.h"
#include "flowmeter.h"
#include "analogio.h"
#include "simpleio.h"

#include <Arduino.h>
#include <avr/sleep.h>

bool Calibration::CalibrationMode()
{
  // test for instruction to enter calibration
  if (!Serial.available()) return false;
  if (Serial.read() != 'C') return false;

  // switch all hardware off
  SimpleIo::switchBuzzer(false);
  SimpleIo::switchSolenoid(false);
  MainsSyncManager::setHeaterPower(0.0);
  MainsSyncManager::setPumpPower(0.0);

  while (Serial.available()) Serial.read();
  Serial.println(F("Please enter a calibration operation:"));
  Serial.println(F("1. Calibrate PCB temperature measurement."));
  Serial.println(F("2. Calibrate a temperature probe."));
  Serial.println(F("3. Calibrate the flow meter."));
  Serial.println(F("4. Calibrate the water level meter."));
  Serial.println(F("5. Calibrate the boiler thermodynamics."));
  Serial.println(F("6. Flush boiler with cold water."));
  Serial.println(F("7. Heat to specified temperature then cool."));
  Serial.println(F("8. Enter a low noise quiescent state."));
  Serial.println(F("X. Exit and return to main program."));
  while (!Serial.available());
  char option = Serial.read();

  if (option == 'X') return true;

  // calibrate ambient temperature measurement
  if (option == '1')
    AnalogIo::calibratePcbTemp();

  // calibrate a temperature probe
  if (option == '2')
    AnalogIo::calibrateTempProbe();

  // calibrate the flow meter
  if (option == '3')
    FlowMeter::calibrate();

  // calibrate the water level meter
  if (option == '4')
    AnalogIo::calibrateWaterLevelMeter();

  // calibrate the boiler thermodynamics
  if (option == '5')
  {
    Serial.println(F("Calibrating boiler thermodynamics."));

    // ensure tank full
    Serial.println(F("Ensure that: 1. Water tank is full. 2. There is a jug to catch water. 3. Steam valve is closed."));
    Serial.println(F("Then hit a key."));
    while(!Serial.available());
    while(Serial.available()) Serial.read();

    // request ambient temp
    Serial.println(F("Please enter the ambient temperature (and ensure water in the tank is at ambient)."));
    while(Serial.available()) Serial.read();
    while(!Serial.available());
    double ambientTemp = Serial.parseFloat();
    Serial.println(ambientTemp);

    // fill boiler
    Serial.println(F("filling boiler"));
    MainsSyncManager::setPumpPower(1.0);
    unsigned long start = millis();
    while(millis() - start < 500 || millis() - start < 5000 && FlowMeter::getFlowRate() >= 1.2);
    MainsSyncManager::setPumpPower(0.0);
    Serial.println(F("filled boiler"));

    Serial.println(F("Raising temperature to 90°C and then resting."));

    // we need to track the total energy injected into the boiler and the integral of (temperature - ambient) dt
    double initialTemp = AnalogIo::getAvgTemperature();
    double temp1 = 0.0;
    double temp2 = initialTemp;
    double totalEnergy1 = 0.0;
    double totalTempTimesTime1 = 0.0;
    double totalEnergy2 = 0.0;
    double totalTempTimesTime2 = 0.0;

    // run the temperature up to 90°C and then wait for a minute to let things stabilise
    unsigned long lastTime = micros();
    unsigned long startTime;
    double power = SAFE_HEATER_POWER;
    for (enum {STATE_HEAT, STATE_COOL1, STATE_COOL2, STATE_END} state = STATE_HEAT; state != STATE_END; )
    {
      MainsSyncManager::setHeaterPower(power);

      // wait and report
      delay(1000);
      Serial.println(temp2);

      // update smoothed temperature
      temp2 = temp2 * 0.5 + AnalogIo::getAvgTemperature() * 0.5;

      // update time
      unsigned long now = micros();
      double deltaTime = (double)(now - lastTime) / 1000000;
      lastTime = now;

      // update integrals
      totalEnergy2 += power * deltaTime;
      totalTempTimesTime2 += (temp2 - ambientTemp) * deltaTime;

      if (state == STATE_HEAT && temp2 >= 90.0)
      {
        power = 0.0;
        state = STATE_COOL1;
        startTime = micros();
      }
      if (state == STATE_COOL1 && micros() - startTime > 100000000)
      {
        temp1 = temp2;
        totalEnergy1 = totalEnergy2;
        totalTempTimesTime1 = totalTempTimesTime2;
        state = STATE_COOL2;
        startTime = micros();
      }
      if (state == STATE_COOL2 && micros() - startTime > 60000000)
        state = STATE_END;
    }

    // calculate boiler parameters
    double powerLossPerDegree = ((temp2 - initialTemp) * totalEnergy1 - (temp1 - initialTemp) * totalEnergy2) / ((temp2 - initialTemp) * totalTempTimesTime1 - (temp1 - initialTemp) * totalTempTimesTime2);
    double internalEnergyPerDegree = (totalEnergy2 - totalTempTimesTime2 * powerLossPerDegree) / (temp2 - initialTemp);
    double shellEnergyPerDegree = internalEnergyPerDegree - SPEC_HEAT_WATER_100 * BOILER_VOLUME;

    // report boiler parameters
    Serial.print(F("BREWHEAD_AMBIENT_XFER_COEFF (in W/K) = "));
    Serial.println(powerLossPerDegree);
    Serial.print(F("SPEC_HEAT_BOILER (in J/K) = "));
    Serial.println(shellEnergyPerDegree);

    Serial.println(F("Switching to flow measure"));
    MainsSyncManager::setHeaterPower(SAFE_HEATER_POWER);
    MainsSyncManager::setPumpPower(1.0);
    SimpleIo::switchSolenoid(true);

    double startVolume = FlowMeter::getTotalVolume();
    double boilerHeatTransferCoeff = 0.0;
    while (FlowMeter::getTotalVolume() - startVolume < 1000)
    {
      // update smoothed temperature and flow rate
      temp2 = temp2 * 0.9 + AnalogIo::getAvgTemperature() * 0.1;
      double flowRate = FlowMeter::getFlowRate();

      // calculate heat transfer coefficient from boiler shell to water
      double joulesPerMl = SAFE_HEATER_POWER / flowRate;
      double waterTemp = ambientTemp + joulesPerMl / SPEC_HEAT_WATER_100;
      boilerHeatTransferCoeff = SAFE_HEATER_POWER / (temp2 - waterTemp);

      Serial.print(temp2);
      Serial.print(",");
      Serial.print(waterTemp);
      Serial.print(",");
      Serial.println(boilerHeatTransferCoeff);
      delay(500);
    }

    MainsSyncManager::setHeaterPower(0.0);
    MainsSyncManager::setPumpPower(0.0);
    SimpleIo::switchSolenoid(false);

    Serial.print(F("BOILER_WATER_XFER_COEFF (in W/K) = "));
    Serial.println(boilerHeatTransferCoeff);
  }

  // flush the boiler with cold water
  if (option == '6')
  {
    // ensure tank full
    Serial.println(F("Ensure that: 1. Water tank is full. 2. There is a jug to catch water."));
    Serial.println(F("Then hit a key."));
    while(!Serial.available());
    delay(1);
    while(Serial.available()) Serial.read();
    Serial.println(F("What percentage power?"));
    while(!Serial.available());
    double power = Serial.parseFloat();
    power = 0.01 * min(100.0, max(1.0, power));

    double startVolume = FlowMeter::getTotalVolume();
    MainsSyncManager::setPumpPower(power);
    SimpleIo::switchSolenoid(true);
    while (FlowMeter::getTotalVolume() - startVolume < 500);
    MainsSyncManager::setPumpPower(0.0);
    SimpleIo::switchSolenoid(false);
  }

  // heat and then cool, reporting temperature curve
  if (option == '7')
  {
    delay(1);
    while(Serial.available()) Serial.read();
    Serial.println(F("Please enter maximum temperature."));
    while(!Serial.available());
    double maxTemp = Serial.parseFloat();
    Serial.println(maxTemp);

    // fill boiler
    Serial.println(F("filling boiler"));
    MainsSyncManager::setPumpPower(1.0);
    unsigned long start = millis();
    while(millis() - start < 500 || millis() - start < 5000 && FlowMeter::getFlowRate() >= 1.2);
    MainsSyncManager::setPumpPower(0.0);
    Serial.println(F("filled boiler"));

    delay(1);
    while(Serial.available()) Serial.read();
    Serial.println(F("Send any character to stop."));
    Serial.println("time,power,probe1,probe2,probe3");

    unsigned long t0 = micros();
    unsigned long tn = t0;
    double power = 0.0;
    MainsSyncManager::setHeaterPower(power);
    for (enum {STATE_COAST, STATE_HEAT, STATE_COOL} state = STATE_COAST; !Serial.available(); )
    {
      // read temperatures
      const double reading1 = AnalogIo::getTemperature(AnalogIo::PROBE1);
      const double reading2 = AnalogIo::getTemperature(AnalogIo::PROBE2);
      const double reading3 = AnalogIo::getTemperature(AnalogIo::PROBE3);

      // report
      Serial.print((tn - t0) / 1000000l);
      Serial.print(",");
      Serial.print(power);
      Serial.print(",");
      Serial.print(reading1);
      Serial.print(",");
      Serial.print(reading2);
      Serial.print(",");
      Serial.println(reading3);

      // safety
      if (isnan(reading1) || isnan(reading2) || isnan(reading3))
      {
        MainsSyncManager::setHeaterPower(0.0);
        Serial.println("Unknown temperature read - aborting.");
        break;
      }

      // coast for five seconds then heat
      if (state == STATE_COAST && tn - t0 >= 5000000l)
      {
        power = SAFE_HEATER_POWER;
        MainsSyncManager::setHeaterPower(power);
        state = STATE_HEAT;
      }

      // when we reach temperature, switch to cooling
      if (state == STATE_HEAT && reading1 >= maxTemp || reading2 >= maxTemp || reading3 >= maxTemp)
      {
        power = 0.0;
        MainsSyncManager::setHeaterPower(power);
        state = STATE_COOL;
      }

      // wait for next tick
      while(micros() - tn < 1000000);
      tn += 1000000;
    }
    MainsSyncManager::setHeaterPower(0);

    while(Serial.available()) Serial.read();
  }

  // go to sleep forever
  if (option == '8')
  {
    Serial.println("Going to sleep forever with all pins in defined and sensible state and all clocks off.");
    Serial.end();

    // stop all interrupts and disable the ones that can wake us (probably going overboard on the pin change interrupt registers, cli() likely covers it)
    cli();
    PCICR &= ~_BV(PCIE2) & ~_BV(PCIE1) & ~_BV(PCIE0);
    PCMSK0 = 0x00;
    PCMSK1 = 0x00;
    PCMSK2 = 0x00;

    // briefly drive all pins to input pullup and then condition specific pins
    // (note the ADC pins won't be affected as the DIDR0 register will have disabled their input circuits)
    DDRB = 0x00;
    PORTB = 0xff;
    DDRC = 0x00;
    PORTC = 0xff;

    // put all digital outputs in quiet but sensible state
    digitalWrite(SOLENOID_PIN, false);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(PUMP_POWER_PIN, false);
    pinMode(PUMP_POWER_PIN, OUTPUT);
    digitalWrite(BOILER_POWER_PIN, false);
    pinMode(BOILER_POWER_PIN, OUTPUT);
    digitalWrite(ST_BOILER_POWER_PIN, false);
    pinMode(ST_BOILER_POWER_PIN, OUTPUT);
    digitalWrite(TEMP_OKAY_LED_PIN, false);
    pinMode(TEMP_OKAY_LED_PIN, OUTPUT);
    digitalWrite(PIEZO_BUZZER_PIN, false);
    pinMode(PIEZO_BUZZER_PIN, OUTPUT);

    // condition digital inputs to be at a defined and sensible voltage
    pinMode(STEAM_SW_PIN, INPUT_PULLUP);
    pinMode(PUMP_SW_PIN, INPUT_PULLUP);
    pinMode(FLOW_METER_PIN, INPUT_PULLUP);
    digitalWrite(MAINS_POLARITY_PIN, false);
    pinMode(MAINS_POLARITY_PIN, OUTPUT);

    // go to sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#ifdef sleep_bod_disable
    sleep_bod_disable();
#endif
    sleep_mode();
  }

  return true;
}
