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

#ifndef COFFEEID_CONSTANTS_H
#define COFFEEID_CONSTANTS_H

#include <Arduino.h>

// Code modules to compile: by selecting fewer modules, it is possible to get the controller or
// calbration code to fit into an ATmega168 or just save upload time whilst developing.
#define CALIBRATION_CODE                          // define to include calibration code
#define CONTROLLER_CODE                           // define to implement the actual controller algorithms
#define REPORTING_CODE                            // define to include status reporting over the serial port

// operating parameters
static const double BREW_SETPOINT = 97.0;       // °C
static const double STEAM_SETPOINT = 166.0;     // °C average boiler temperature (bearing in mind element sides are a hotter and other sides cooler)
static const double MAX_FLOW_RATE = 10.0;       // ml/s well over what the pump can produce just to be sure
static const double BREW_FLOW_RATE_2CUP = 3.5;  // ml/s
static const double BREW_FLOW_RATE_1CUP = 2.5;  // ml/s half of BREW_FLOW_RATE_2CUP would be good, but it's a little close to the flow meter's min rate
static const double MIN_FLOW_RATE = 2.0;        // ml/s where the flow rate begins to be able to be controlled
static const double BREW_VOLUME_2CUP = 105.0;   // ml
static const double BREW_VOLUME_1CUP = 60.0;    // ml
static const double MIN_RESERVOIR_VOLUME = 100.0;// ml
static const unsigned INACTIVITY_TIMEOUT = 1200;// seconds with no activity before stopping heating activating the buzzer (20 minutes)
static const unsigned long BACKFLUSH_STALL_TIME = 1000000l; // microseconds to wait for while pump is stalled before flushing
static const unsigned long BACKFLUSH_FLUSH_TIME = 2500000l; // microseconds to wait for while pump is flushing before restarting
static const double STEAM_PUMP_PULSE_TEMP = 140.0;// temperature at which pulsing the pump begins in steam mode.
static const double STEAM_PUMP_PULSE_DUTY = 0.12;// duty cycle for pulsing the pump in steam mode.

// physical charcteristics of the AVR's internal temperature probe and circuitry
static const double PCBTEMP_OFFSET = 352.0;               // °C the offset to subtract to get the PCB's temperature once the AVR's self-heating is complete
static const double BANDGAP_VOLTAGE = 1.1;                // V nominal bandgap voltage from the datasheet
static const double AVRTEMP_FACTOR = 1.0e3;               // °C/V the ratio of ADC reading to temperature, from the datasheet
static const double AREF_VOLTAGE = 1.8;                   // V the voltage on the AREF pin

// physical characteristics of temperature probes 1 to 3 and their circuitry
static const double PT100_A = -0.00005775;
static const double PT100_B = 0.39083;
static const double PT100_C = 100.0;
static const double KTYPE_TEMPS[] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200};                                                   // °C
static const double KTYPE_VOLTAGES[] = {0,0.397,0.798,1.203,1.612,2.023,2.436,2.851,3.267,3.682,4.096,4.509,4.92,5.328,5.735,6.138,6.54,6.941,7.34,7.739,8.138};  // mV
static const double VCC_VOLTAGE = 5.09;                   // V the arduina 5V regulator may not yield exactly 5V
//#define PROBE1_KTYPE
//static const double VOLTAGE_FACTOR1 = 1.8 / 1024 / (3.3e6 / 9.88e3);  // V linear function mapping from ADC reading to probe voltage
//static const double VOLTAGE_OFFSET1 = 0.129e-3;                       // ...
#define PROBE1_PT100
static const double RESISTANCE_FACTOR1 = 0.05;              // linear function mapping from ADC reading to resistance
static const double RESISTANCE_OFFSET_LOWRANGE1 = 93.77;    // ...
static const double RESISTANCE_OFFSET_HIGHRANGE1 = 116.85;  // ...
#define PROBE2_PT100
static const double RESISTANCE_FACTOR2 = 0.05;              // linear function mapping from ADC reading to resistance
static const double RESISTANCE_OFFSET_LOWRANGE2 = 94.05;    // ...
static const double RESISTANCE_OFFSET_HIGHRANGE2 = 117.54;  // ...
#define PROBE3_NTC
static const double VOLTAGE_FACTOR3 = 3.1e-3;               //  linear function mapping from ADC reading to voltage
static const double VOLTAGE_OFFSET_LOWRANGE3 = 2.16;        // ...
static const double VOLTAGE_OFFSET_HIGHRANGE3 = 2.7;        // ...
static const double SERIES_R3 = 4.68e3;                     // Ohms the resistance of the resistor in series with the probe
static const double R25_3 = 106e3;                          // Ohms the resistance of the NTC probe at 25°C
static const double BETA3 = 3950.0;                         // the beta value of the NTC probe

// characterisitics of the flow meter
static const double FLOW_METER_DEBOUNCE_TIME = 0.02; // any state change within 20ms of the last is considered a bounce error (from pump vibration)

// characteristics of the level meter probe
static const long LEVEL_PROBE_FULL_VALUE = 905;  // ADC reading when reservoir full (approximate - very dependent on weather)
static const long LEVEL_PROBE_EMPTY_VALUE = 526; // ADC reading when reservoir empty
static const double RESERVOIR_VOLUME = 1500.0;    // ml total vlume of reservoir

// characteristics of the temperature controller
static const double MAX_HEATER_POWER = 2700.0;             // W at 240V (43 Ohm elements in semi-parallel)
static const double SAFE_HEATER_POWER = 2500.0;            // W power beyond which I dare not drive the beast
static const double ELEMENT_MAX_TEMPERATURE = 190.0;       // °C the thermal fuses melt at 172°C and elements are about 20K hotter than fuse mountings
static const double SHELL_MAX_TEMPERATURE = 152.0;         // °C the temp probe circuit maxes out at about 155°C - don't exceed it or we're flying blind
static const double AMBIENT_TEMPERATURE = 20.0;            // °C this does not need to be exact
static const double RESERVOIR_TEMPERATURE = 20.0;          // °C the reservoir gets heated a little by the machine over time - model this?
static const double LATENT_HEAT_VAPORISATION_100 = 2257.0; // J/g latent heat of vaporisation of water at 100°C
static const double SPEC_HEAT_WATER_100 = 4.216;           // J/g/K specific heat of water at 100°C
static const double SPEC_HEAT_ALUMINIUM = 0.9;             // J/g/K specific heat of aluminium (i.e. boiler shell)
static const double SPEC_HEAT_BRASS = 0.38;                // J/g/K specific heat of brass (i.e. brew head)
static const double BOILER_VOLUME = 100.0;                 // ml volume of water in boiler when full (I measured it)
static const double MASS_BOILER_SHELL = 609.0;             // g mass of the aluminium boiler shell (I measured it)
static const double MASS_BREW_HEAD = 1172.0;               // g mass of the brew head (I measured it)
static const double MASS_PORTAFILTER = 450.0;              // g mass of the brew head (I measured it)
static const double HEAT_CAPACITY_BODY = 395.0;            // J/K heat capacity of the body/housing of the machine (i.e. what is lost once-off during startup)
static const double BREWHEAD_AMBIENT_XFER_COEFF = 0.55;    // W/K power lost from brew head to ambient air
static const double BOILER_WATER_XFER_COEFF_NOFLOW = 14.7; // W/K rate at which boiler shell heats water per °C difference in boiler and water temperature
static const double BOILER_WATER_XFER_COEFF_STEAM = 25.0;  // W/K ditto for when steam is flowing
static const double BOILER_BREWHEAD_XFER_COEFF = 3.6;      // W/K ditto for the brewhead (calculated from measuring brewhead temperature at 60s of full power=14.2K+ambient and boiler temp=67.21K+ambient and rate of change of brewhead temp=0.43K/s)
static const double ELEMENT_SHELL_XFER_COEFF = 14.0;       // W/K rate at which heat transfers from element half of boiler to shell half
static const double BOILER_BODY_XFER_COEFF = 1.8;          // W/K rate at which heat transfers into the body/housing of the machine

// characteristics of the pump controller
static const double PUMP_PID_P = -0.05;         // a low enough value to avoid oscillation even with no back-pressure

// characteristics of the boiler triac heat system (see http://www.farnell.com/datasheets/1723975.pdf)
static const double MAX_TRIAC_POWER = 11.0;                     // W the power dissipated when triac is full on, i.e. at MAX_HEATER_POWER, from datasheet
static const double TRIAC_JUNCTION_CASE_XFER_CEOFF = 1.0 / 1.7; // W/K heat transfer coefficient from junction to triac case from datasheet
static const double TRIAC_CASE_AMBIENT_XFER_CEOFF = 1.0 / 5.0;  // W/K measured
static const double TRIAC_JUNCTION_AMBIENT_XFER_COEFF = 1.0 / (1.0 / TRIAC_JUNCTION_CASE_XFER_CEOFF / 2.0 + 1.0 / TRIAC_CASE_AMBIENT_XFER_CEOFF);
static const double TRIAC_HEAT_CAPACITY = 32.7;                 // J/K heat capacity of triac and heatsink measured
static const double TRIAC_MAX_TEMPERATURE = 115.0;              // °C max junction temperature from datasheet minus 10°C safety margin

// pin mappings
static const uint8_t SOLENOID_PIN = 8;         //
static const uint8_t PUMP_POWER_PIN = 3;       // = OC2B
static const uint8_t BOILER_POWER_PIN = 2;     //
static const uint8_t ST_BOILER_POWER_PIN = 9;  //
static const uint8_t STEAM_SW_PIN = 4;         //
static const uint8_t PUMP_SW_PIN = 10;         // = SS (needed for SPI slave)
static const uint8_t TEMP_OKAY_LED_PIN = 14;   // = PC0
static const uint8_t FLOW_METER_PIN = 15;      // = PC1 = PCINT9
#define FLOW_METER_PCINT PCINT9
static const uint8_t MAINS_POLARITY_PIN = 7;   // = PCINT23
#define MAINS_POLARITY_PCINT PCINT23
static const uint8_t PIEZO_BUZZER_PIN = 6;     //
static const uint8_t TEMP_LOW_RANGE_PIN = 5;   //
static const uint8_t TEMP_PROBE_1_ADCPIN = 6;  //
static const uint8_t TEMP_PROBE_2_ADCPIN = 7;  //
static const uint8_t TEMP_PROBE_3_ADCPIN = 2;  //
static const uint8_t LEVEL_METER_ADCPIN = 3;   //
static const uint8_t TEMP_INTERNAL_ADCPIN = 8; //
static const uint8_t BANDGAP_ADCPIN = 14;      //

// characteristics of the zero crossover circuit
static const double MAINS_CROSS_LATENCY = 60e-6;   // s the time after mains zero crossing to +ve when the pin goes low (at +1.9V on the mains)
static const double MAINS_CYCLE_LENGTH = 20e-3;    // s 20 milliseconds for a 50Hz supply

#endif
