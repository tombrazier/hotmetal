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

#include "constants.h"
#include "mainssync.h"
#include "flowmeter.h"
#include "analogio.h"
#include "simpleio.h"
#include "calibration.h"

// NOTE: the timing assumes the CPU main clock speed is 8Mhz or 16Mhz
// NOTE: serial port runs at 38400
// NOTE: your starting point is constants.h, which controls everything

// TODO:
// -implement steam boiler input / output
// -protect pump from overheating by preventing it from running for too long or on an empty tank (zero flow)
// -if there is a temperature probe problem, use a beep warning code
// -model water xfer coeff getting as high as 40 when at full flow
// -measure mains AC voltage to better control power
// -single cup steam mode: lower temperature?
// -track boiler volume (it's full when pump stalls and empties in steam mode at rate of power/LATENT_HEAT_VAPORISATION_100) and fill when needed instead of pulsing the pump
// -inject water more gently in steam mode (but how? maybe use full mains pulses but not every pulse like boiler?)
// -should we model heat transfer from water to brewhead and vice versa? (maybe not, it isn't right at present but it works)

void setup()
{
  // All pins should initially be high-impedance inputs. Any pins not explicitly initialised
  // below (i.e. unconnected pins) should be set to pull-up inputs. We set this up, but
  // disable pull-ups until the end of the initialisation.
  MCUCR |= _BV(PUD);
  DDRB = 0x00;
  DDRC = 0x00;
  DDRD = 0x00;
  PORTB = 0xff;
  PORTC = 0xff;
  PORTD = 0xff;

  // set up all the hardware
  MainsSyncManager::initialise();
  FlowMeter::initialise();
  AnalogIo::initialise();
  SimpleIo::initialise();

  // now setup is done, enable pullups
  MCUCR &= ~_BV(PUD);

  // set up comms
  Serial.begin(38400);

  // wait for mains zero-crossing to stabilise, but don't hold off starting the heating process
  while(!MainsSyncManager::ready())
  {
    double temperature = AnalogIo::getAvgTemperature();
    MainsSyncManager::setHeaterPower(!isnan(temperature) && temperature < BREW_SETPOINT ? SAFE_HEATER_POWER : 0.0);
  }
  MainsSyncManager::setHeaterPower(0.0);
}

void loop()
{
  // print hello world message
  Serial.println();
  Serial.println(F("Arduino Gaggia controller."));
#ifdef CALIBRATION_CODE
  Serial.println(F(" Type 'C' to enter calibration mode."));
#endif // CALIBRATION_CODE

  // clever trick: on startup, the pump switch tells us if we're in 1 cup mode and the steam switch tells us whether we're in backflush mode
  enum {BREW_1CUP, BREW_2CUP, BACKFLUSH} brewMode = BREW_2CUP;
  brewMode = SimpleIo::readPumpSwitch() ? BREW_1CUP : SimpleIo::readSteamSwitch() ? BACKFLUSH : BREW_2CUP;
  while(SimpleIo::readPumpSwitch()) delay(1);
  while(SimpleIo::readSteamSwitch()) delay(1);

  // initial value for temperature, etc.
  double shellTemp = AnalogIo::getAvgTemperature();
  if (isnan(shellTemp)) shellTemp = AMBIENT_TEMPERATURE;
  double ambientTemp = min(shellTemp, AMBIENT_TEMPERATURE);
  double elementTemp = shellTemp;
  double waterTemp = shellTemp;
  // based on heat transfer coefficients to and from brewhead, we can caluculate its steady state temp - use that
  double brewHeadTemp = (ambientTemp * BREWHEAD_AMBIENT_XFER_COEFF + shellTemp * BOILER_BREWHEAD_XFER_COEFF) / (BREWHEAD_AMBIENT_XFER_COEFF + BOILER_BREWHEAD_XFER_COEFF);
  double bodyTemp = shellTemp;
  // who knows what the triac temp is - but we can guess if we assume the triac gets to max temp when in steam mode for a long time)
  const double pcbTemp = AnalogIo::getPcbTemperature();
  double triacTemp = pcbTemp + (TRIAC_MAX_TEMPERATURE - pcbTemp) * (shellTemp - ambientTemp) / (STEAM_SETPOINT - ambientTemp);
  double pumpPowerRate = 0.0;
  double heaterPower = 0.0;

  // class to record the run state of the machine
  class State
  {
    public:
      typedef enum {BEGIN, PREHEAT_FILL, PREHEAT_HEAT, PREHEAT_VENT, PREHEAT_COOL, READY, STEAM, BACKFLUSH, BREW, INACTIVE} STATE_TYPE;
    private:
      STATE_TYPE    m_state;
      unsigned long m_lastChangeTime;
    public:
      State()
      {
        m_lastChangeTime = millis();
        m_state = BEGIN;
      }
      // set the state and return whether it had changed
      bool setState(STATE_TYPE state)
      {
        if (m_state == state) return false;
        m_lastChangeTime = millis();
        m_state = state;
        return true;
      }
      STATE_TYPE getState()
      {
        return m_state;
      }
      unsigned long millisInState()
      {
        return millis() - m_lastChangeTime;
      }
  };
  State state;

  while (true)
  {
#ifdef CALIBRATION_CODE
    // if necessary, run calibration routines
    if (Calibration::CalibrationMode()) break;
#endif // CALIBRATION_CODE

#ifdef CONTROLLER_CODE
    // read physical state
    bool steamOn = SimpleIo::readSteamSwitch();
    bool pumpOn = SimpleIo::readPumpSwitch();
    double flowRate = FlowMeter::getFlowRate();
    double waterLevel = AnalogIo::getWaterLevel();

    // control variables
    double flowSetpoint = 0.0;
    double tempSetpoint = 0.0;

    // main state machine which decides what activity we're doing and sets solenoid, flow, temperature and buzzer accordingly...
    if (state.getState() == State::BEGIN)
    {
      // at startup, go into the preheat routine unless already hot or backflushing
      if (fabs(waterTemp - BREW_SETPOINT) >= 5.0 && brewMode != BACKFLUSH)
        state.setState(State::PREHEAT_FILL);
      else
        state.setState(State::READY);
    }

    // temperature stabilisation routine: drive water through the system once at temperature (but cancel preheat if any button is pressed)
    if (state.getState() > State::BEGIN && state.getState() < State::READY)
    {
      // cancel preheat if any button is pressed
      if (steamOn || pumpOn)
        state.setState(State::READY);

      SimpleIo::switchBuzzer(false);
      SimpleIo::setTempOkayLed(false);

      if (state.getState() == State::PREHEAT_FILL)
      {
        // make sure the boiler is full by pumping against a closed solenoid valve until there is no flow
        SimpleIo::switchSolenoid(false);
        flowSetpoint = MAX_FLOW_RATE;
        tempSetpoint = STEAM_SETPOINT;

        // pump for at least half a quarter before checking flow rate and then move on if flow is less than 1.2 ml/s
        // NOTE: below 1.2 ml/s the flow meter can give very different results depending on where any bubbles
        // are in the system - indeed, it might never measure below 1 ml/s even with the minor dribble leak from the steam wand.
        // NOTE: it is an error condition if we pump for too long (maybe steam valve is open?) - give up after 20s, which
        // ought to be long enough to fill the boiler.
        if (state.millisInState() > 250 && flowRate < 1.2 || state.millisInState() > 20000)
          state.setState(State::PREHEAT_HEAT);
      }
      if (state.getState() == State::PREHEAT_HEAT)
      {
        // Open the solenoid to release pressure and heat up to the system preheat setpoint.
        // Releasing the pressure now is very important - otherwise later on we get massive and
        // unpredictable cooling as water flashes in the low pressure pulse.
        // But don't open the solenoid for too long - we don't want to cause unneccessary cooling by evaporation.
        SimpleIo::switchSolenoid(state.millisInState() < 1000);
        flowSetpoint = 0.0;
        // heat as fast as we can (but don't overshoot the steam setoint)
        tempSetpoint = STEAM_SETPOINT;
        // heat until the water is at boiling point
        if (waterTemp >= 100.0)
          state.setState(State::PREHEAT_VENT);
      }
      if (state.getState() == State::PREHEAT_VENT)
      {
        // let the steam vent until we think the brewhead is up to its asymptotic temperature
        SimpleIo::switchSolenoid(true);
        flowSetpoint = 0.0;
        tempSetpoint = STEAM_SETPOINT;
        if (brewHeadTemp >= 85.0)
          state.setState(State::PREHEAT_COOL);
      }
      if (state.getState() == State::PREHEAT_COOL)
      {
        // flow water through to cool the water and complete heating the brew head
        // stop shortly before the total boiler + water energy is right because we'll
        // need the temperatures to equalise and in the mean time, there is loss to ambient
        // experience shows 4kJ above target is where we should stop
        SimpleIo::switchSolenoid(true);
        flowSetpoint = MIN_FLOW_RATE;
        tempSetpoint = BREW_SETPOINT;
        // stop when total energy of water and boiler is right
        if ((waterTemp - BREW_SETPOINT) * SPEC_HEAT_WATER_100 * BOILER_VOLUME + ((shellTemp + elementTemp) / 2.0 - BREW_SETPOINT) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL <= 4000.0)
          state.setState(State::READY);
      }
    }
    // steam state (entering steam state takes precedence over entering brew state)
    if (state.getState() == State::READY && steamOn || state.getState() == State::STEAM)
    {
      bool stateChanged = state.setState(State::STEAM);

      SimpleIo::switchBuzzer(false);

      // set the temperature to the steam temp
      tempSetpoint = STEAM_SETPOINT;

      SimpleIo::setTempOkayLed(fabs((elementTemp + shellTemp) / 2.0 - tempSetpoint) < 0.5);

      SimpleIo::switchSolenoid(false);

      // pulse pump in steam mode once a decent temperature is attained
      static bool pulsePump = false;
      if (stateChanged) pulsePump = false;
      pulsePump = pulsePump || shellTemp > STEAM_PUMP_PULSE_TEMP;
      flowSetpoint = pulsePump && (micros() % 5000000 < (unsigned long)5000000 * STEAM_PUMP_PULSE_DUTY) ? MAX_FLOW_RATE : 0.0;

      if (!steamOn)
        state.setState(State::READY);
    }
    // in backflush mode, repeatedly drive the pump until it stalls, then stop pump and close solenoid for a few seconds
    if (state.getState() == State::READY && pumpOn && brewMode == BACKFLUSH || state.getState() == State::BACKFLUSH)
    {
      bool stateChanged = state.setState(State::BACKFLUSH);

      SimpleIo::switchBuzzer(false);

      tempSetpoint = BREW_SETPOINT;

      SimpleIo::setTempOkayLed(fabs(waterTemp - tempSetpoint) < 0.5);

      static unsigned long stateChangeTime = 0;
      static bool pumping = false;
      if (stateChanged)
        pumping = true;

      if (stateChanged || pumping && flowRate > 1.2)
        stateChangeTime = micros() + BACKFLUSH_STALL_TIME;

      if ((long)micros() - (long)stateChangeTime > 0)
      {
        pumping = !pumping;
        stateChangeTime = micros() + (pumping ? BACKFLUSH_STALL_TIME : BACKFLUSH_FLUSH_TIME);
      }

      SimpleIo::switchSolenoid(pumping);
      flowSetpoint = pumping ? MAX_FLOW_RATE : 0.0;

      if (!pumpOn)
        state.setState(State::READY);
    }
    // in pump mode with 1 or 2 cup flow rate, we let the PID controller do the driving
    if (state.getState() == State::READY && pumpOn && brewMode != BACKFLUSH || state.getState() == State::BREW)
    {
      bool stateChanged = state.setState(State::BREW);

      SimpleIo::switchBuzzer(false);

      tempSetpoint = BREW_SETPOINT;

      SimpleIo::setTempOkayLed(fabs(waterTemp - tempSetpoint) < 0.5);

      // calculate total volume of shot
      static double brewStopVolume = 0.0;
      if (stateChanged)
        brewStopVolume = FlowMeter::getTotalVolume() + (brewMode == BREW_2CUP ? BREW_VOLUME_2CUP : BREW_VOLUME_1CUP);

      // now control flow and solenoid based on whether enough has been pumped through
      flowSetpoint = brewMode == BREW_2CUP ? BREW_FLOW_RATE_2CUP : BREW_FLOW_RATE_1CUP;

      // At the end of the shot, request a very low flow rate so the pressure drops
      // right off and the backflush will be negligible (doesn't really matter if we can
      // control to this, just that it is still slightly positive flow). Once the flow
      // has dropped enough, end the shot and close the solenoid valve.
      if (FlowMeter::getTotalVolume() > brewStopVolume - 5.0)
      {
        flowSetpoint = 0.25;
        if (flowRate < 0.3)
        {
          flowSetpoint = 0.0;

          // having reached the end of the shot, jump out of this state if the steam button is
          // pressed so we can enter steam state
          if (steamOn)
            state.setState(State::READY);
        }
      }
      SimpleIo::switchSolenoid(flowSetpoint > 0.0);

      if (!pumpOn)
        state.setState(State::READY);
    }
    // Guard against long periods of unuse - has the machine been left on accidentally?
    // We'll allow this for twice as long in backflush mode.
    // Also guard against the TRIAC temperature getting to unsafe levels that could cause thermal runaway which
    // should never happen unless the machine is in steam mode for a very very long time.
    if (state.getState() != State::BACKFLUSH && state.millisInState() > (unsigned long)INACTIVITY_TIMEOUT * 1000 ||
      state.millisInState() > (unsigned long)INACTIVITY_TIMEOUT * 1000 * 2 ||
      triacTemp >= TRIAC_MAX_TEMPERATURE ||
      state.getState() == State::INACTIVE)
    {
      static unsigned char switches;
      if (state.setState(State::INACTIVE))
        switches = (steamOn ? 0x02 : 0x00) | (pumpOn ? 0x01 : 0x00);

      SimpleIo::switchBuzzer(true);
      SimpleIo::switchSolenoid(false);
      SimpleIo::setTempOkayLed(false);
      flowSetpoint = 0.0;
      tempSetpoint = 0.0;

      // only exit INACTIVE state if switches change
      if (switches != ((steamOn ? 0x02 : 0x00) | (pumpOn ? 0x01 : 0x00)))
        state.setState(State::PREHEAT_FILL);
    }
    if (state.getState() == State::READY)
    {
      // complain if the reservoir is empty
      if (waterLevel < MIN_RESERVOIR_VOLUME)
        SimpleIo::switchBuzzer((millis() / 500) % 2 == 0);
      else
        SimpleIo::switchBuzzer(false);

      SimpleIo::switchSolenoid(false);
      flowSetpoint = 0.0;
      tempSetpoint = BREW_SETPOINT;

      SimpleIo::setTempOkayLed(fabs(waterTemp - tempSetpoint) < 0.5);
    }

    // control the power to the pump based on the flow rate setpoint
    if (flowSetpoint <= 0.0)
      MainsSyncManager::setPumpPower(0.0);
    else
    {
      static unsigned long lastControlTime = 0;
      double deltaTime = (double)(micros() - lastControlTime) / 1000000.0;
      double pumpPower = MainsSyncManager::getPumpPower();

      // update power 50 times per second (i.e. about once every mains cycle)
      if (deltaTime >= 0.02)
      {
        lastControlTime += deltaTime * 1000000;

        // express temperature as an error from flowSetpoint and pump_pid, which drives pumpPowerRate directly
        double error = flowRate - flowSetpoint;
        pumpPowerRate = fabs(error) * error * PUMP_PID_P;
        pumpPower += deltaTime * pumpPowerRate;
        pumpPower = min(1.0, max(0.0, pumpPower));

        // set pump power
        MainsSyncManager::setPumpPower(pumpPower);
      }
    }

    // every approx 1 second go on to manage temperature (or do so immediately if the activity has changed)
    {
      static unsigned long lastBoilerPidTime = 0;
      double deltaTime = (double)(micros() - lastBoilerPidTime) / 1000000.0;
      static double lastTempSetpoint = 0.0;
      if (lastTempSetpoint == tempSetpoint && deltaTime < 1.0) continue;
      lastTempSetpoint = tempSetpoint;
      lastBoilerPidTime += deltaTime * 1000000;

      // handle reading errors by switching the heater off and moving on
      double readTemperature = AnalogIo::getAvgTemperature();
      if (isnan(readTemperature) || readTemperature < 0.0 || readTemperature > 200.0)
      {
        Serial.println(F("Failed to read sensible temperature"));
        heaterPower = 0.0;
        MainsSyncManager::setHeaterPower(heaterPower);
      }
      else
      {
        // instead of using the temperature as read, we model the total energy and use the read value in place of one of the variables
        // this helps greatly because there is about 8s latency between applying a step change to the power and seeing a result

        // How much heat is lost to water flowing out?
        // Assumptions: 1. Water flows out at the same rate as it flows in.
        //              2. Water flowing in is at ambient temperature.
        //              3. Water enthalpies at 100°C are close enough for the other temperatures at which we operate.
        //              4. Density of water flowing in is 1;
        double waterToFlowPower = flowRate * (waterTemp - RESERVOIR_TEMPERATURE) * SPEC_HEAT_WATER_100;

        // How much power is lost to the atmosphere from the brew head?
        double brewHeadToAmbientPower = (brewHeadTemp - ambientTemp) * BREWHEAD_AMBIENT_XFER_COEFF;

        // How much power is transferred from the boiler to the water?
        double shellToWaterPower = (shellTemp - waterTemp) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;
        double elementToWaterPower = (elementTemp - waterTemp) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;

        static bool steamValveOpen = false;
        if (steamValveOpen || flowRate > 1.0)
        {
          shellToWaterPower = (shellTemp - waterTemp) * BOILER_WATER_XFER_COEFF_STEAM / 2.0;
          elementToWaterPower = (elementTemp - waterTemp) * BOILER_WATER_XFER_COEFF_STEAM / 2.0;
        }

        double steamHeatLoss = 0.0;
        if (SimpleIo::solenoidOn() || steamValveOpen)
        {
          // we lose exactly the right amount of heat through vaporisation that the water temp ends up being 100°C
          // which won't be exactly right - the back pressure through the nozzle will mean it's a little more than 100°C
          steamHeatLoss = (waterTemp - 100.0) * (SPEC_HEAT_WATER_100 * BOILER_VOLUME) / deltaTime + shellToWaterPower + elementToWaterPower - waterToFlowPower;
          steamHeatLoss = max(0.0, steamHeatLoss);
        }

        // How much power is transferred from the boiler to the brew head?
        double shellToBrewHeadPower = (shellTemp - brewHeadTemp) * BOILER_BREWHEAD_XFER_COEFF / 2.0;
        double elementToBrewHeadPower = (elementTemp - brewHeadTemp) * BOILER_BREWHEAD_XFER_COEFF / 2.0;
        double waterFlowToBrewHeadPower = (waterTemp - brewHeadTemp) * flowRate * SPEC_HEAT_WATER_100;
        if (brewHeadTemp < 100.0) waterFlowToBrewHeadPower += steamHeatLoss;

        double elementToShellPower = (elementTemp - shellTemp) * ELEMENT_SHELL_XFER_COEFF;

        double shellToBodyPower = (shellTemp - bodyTemp) * BOILER_BODY_XFER_COEFF / 2.0;
        double elementToBodyPower = (elementTemp - bodyTemp) * BOILER_BODY_XFER_COEFF / 2.0;

        // Now work out the temperature, which comes from power that didn't go into heat loss or heating the incoming water.
        brewHeadTemp += deltaTime * (shellToBrewHeadPower + elementToBrewHeadPower + waterFlowToBrewHeadPower - brewHeadToAmbientPower) / (SPEC_HEAT_BRASS * (MASS_BREW_HEAD + MASS_PORTAFILTER));
        waterTemp += deltaTime * (shellToWaterPower + elementToWaterPower - waterToFlowPower - steamHeatLoss) / (SPEC_HEAT_WATER_100 * BOILER_VOLUME);
        shellTemp += deltaTime * (elementToShellPower - shellToBrewHeadPower - shellToWaterPower - shellToBodyPower) / (SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0);
        elementTemp += deltaTime * (heaterPower - elementToShellPower - elementToBrewHeadPower - elementToWaterPower - elementToBodyPower) / (SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0);
        bodyTemp += deltaTime * (shellToBodyPower + elementToBodyPower) / HEAT_CAPACITY_BODY;

        // every two seconds, work out the average power by which shellTemp diverged from readTemperature
        static double accumulationPeriod = 0.0;
        static double shellTempAccumulatedError = 0.0;
        accumulationPeriod += deltaTime;
        shellTempAccumulatedError += readTemperature - shellTemp;
        shellTemp = readTemperature;

        if (waterTemp <= 95.0)
        {
          steamValveOpen = false;
          accumulationPeriod = 0.0;
          shellTempAccumulatedError = 0.0;
        }
        else if (accumulationPeriod >= 2.0)
        {
          double divergencePower = shellTempAccumulatedError * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0 / accumulationPeriod;
          if (steamValveOpen && divergencePower > 100.0 || !steamValveOpen && divergencePower < -100.0)
            steamValveOpen = !steamValveOpen;
          accumulationPeriod = 0.0;
          shellTempAccumulatedError = 0.0;
        }

        // model the temperature of the control triac
        const double pcbTemp = AnalogIo::getPcbTemperature();
        triacTemp += (heaterPower / MAX_HEATER_POWER * MAX_TRIAC_POWER - (triacTemp - pcbTemp) * TRIAC_JUNCTION_AMBIENT_XFER_COEFF) * deltaTime / TRIAC_HEAT_CAPACITY;

        double desiredAverageShellTemp = tempSetpoint;
        if (tempSetpoint < 100.0)
        {
          // we want the water to reach target temperature in the next 15s so calculate the necessary average temperature of the shell
          double desiredWaterInputPower = (tempSetpoint - waterTemp) * SPEC_HEAT_WATER_100 * BOILER_VOLUME / 15.0;
          desiredWaterInputPower += waterToFlowPower;
          desiredAverageShellTemp = waterTemp + desiredWaterInputPower / BOILER_WATER_XFER_COEFF_NOFLOW;
          if (flowRate > 1.0)
            desiredAverageShellTemp = waterTemp + desiredWaterInputPower / 25.0;
          // now clip the temperature so that it won't take more than 20s to lose excess heat to ambient
          double maxStableAverageShellTemp = (brewHeadToAmbientPower * 20.0 - (waterTemp - tempSetpoint) * SPEC_HEAT_WATER_100 * BOILER_VOLUME) / SPEC_HEAT_ALUMINIUM / MASS_BOILER_SHELL + tempSetpoint;
          desiredAverageShellTemp = min(desiredAverageShellTemp, maxStableAverageShellTemp);
        }

        // arrange heater power so that the average boiler energy will be correct in 2 seconds (if possible)
        // the error term handles boiler shell and water - other known power sinks are added explicitly
        double error = (shellTemp - desiredAverageShellTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0;
        error += (elementTemp - desiredAverageShellTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0;
        heaterPower = shellToBrewHeadPower + elementToBrewHeadPower + shellToBodyPower + elementToBodyPower + shellToWaterPower + elementToWaterPower - error / 2.0;

        // keep power level safe and sane (where it would take two seconds to get triac or elements over max temp and five seconds to get shell over max temp)
        double maxAllowableElementToShellPower = (SHELL_MAX_TEMPERATURE - shellTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 5.0 + shellToWaterPower + shellToBrewHeadPower + shellToBodyPower;
        double maxAllowableElementTemp = maxAllowableElementToShellPower / ELEMENT_SHELL_XFER_COEFF + shellTemp;
        maxAllowableElementTemp = min(maxAllowableElementTemp, ELEMENT_MAX_TEMPERATURE);
        double maxAllowablePowerForElement = (maxAllowableElementTemp - elementTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0 + elementToShellPower + elementToWaterPower + elementToBrewHeadPower + elementToBodyPower;
        heaterPower = min(SAFE_HEATER_POWER, heaterPower);
        heaterPower = min(maxAllowablePowerForElement, heaterPower);
        heaterPower = max(0.0, heaterPower);

        MainsSyncManager::setHeaterPower(heaterPower);
      }
    }

    // print column headers to the serial port periodically
    static int rowCount = 0;
    if (rowCount == 0)
    {
      Serial.println(F("time\tsteam \tpump  \theat \telemnt\tshell \twater\tb. head\tPCB  \ttriac\tPROBE3\tAC     \tpump \tdpump \twater\tvolume\tflow \tflow"));
      Serial.println(F("    \tswitch\tswitch\tpower\ttemp  \ttemp  \ttemp \ttemp   \ttemp \ttemp \ttemp  \tcycle  \tpower\tpwr/dt\tlevel\t      \tsetpt\trate"));
    }
    rowCount = (rowCount + 1) % 30;

#ifdef REPORTING_CODE
    // print the data to any computer that happens to be listening
    Serial.print((double)micros() / 1000000.0);
    Serial.print("\t");
    Serial.print(steamOn ? "on" : "off");
    Serial.print("\t");
    Serial.print(pumpOn ? "on" : "off");
    Serial.print("\t");
    Serial.print(heaterPower);
    Serial.print("\t");
    Serial.print(elementTemp);
    Serial.print("\t");
    Serial.print(shellTemp);
    Serial.print("\t");
    Serial.print(waterTemp);
    Serial.print("\t");
    Serial.print(brewHeadTemp);
    Serial.print("\t");
    Serial.print(AnalogIo::getPcbTemperature());
    Serial.print("\t");
    Serial.print(triacTemp);
    Serial.print("\t");
    Serial.print(AnalogIo::getTemperature(AnalogIo::PROBE3));
    Serial.print("\t");
    Serial.print(MainsSyncManager::getMainsCycleLength() * 1000);
    Serial.print("\t");
    Serial.print(MainsSyncManager::getPumpPower());
    Serial.print("\t");
    Serial.print(pumpPowerRate);
    Serial.print("\t");
    Serial.print(waterLevel);
    Serial.print("\t");
    Serial.print(FlowMeter::getTotalVolume());
    Serial.print("\t");
    Serial.print(flowSetpoint);
    Serial.print("\t");
    Serial.print(flowRate);
    Serial.println();
#endif // REPORTING_CODE
#endif // CONTROLLER_CODE
  }
}
