// Copyright Eli Reed (sailedeer), released under GPLv3
// November 18th, 2019
//
// Flight controller for MiniMEE mission. Maintains state of
// payload, facilitates communication with New Shepard,
// and controls experimentation.

// Bonk Rewrite, September 20, 2020

#include <SPI.h>
#include <Wire.h>
#include "SdFat.h"
#include <BonkFramework.h>

// pin configuration macros
// TODO: going to need to update these defines
// will depend on what the chip select will end up being
// and what containment pins
#define CHIP_SELECT 4

#define PUMP_POWER 0
#define PUMP_1 1
#define PUMP_2 2

#define SOL_1 3
#define SOL_2 4
#define SOL_3 5

#define FLUID_SENSOR_1 7
#define FLUID_SENSOR_2 8

#define EXPERIMENT 9

// TODO: change this for the actual sensor we're using
#define TMP411_ADDRESS 0b1001101

// how long to wait before taking a measurement (ms)
#define LOG_TIME_CUTOFF 250

// how long to wait before checking fluid sensor
#define DRAIN_TIME_CUTOFF 250

// how many times do we clean
#define RINSES 2

enum action {
  IDLE,
  START_SULF,
  START_WATER,
  STOP_SULF,
  STOP_WATER,
  CHECK_FULL,
  START_DRAIN,
  CHECK_DRAIN,
  STOP_DRAIN,
  DRAIN_WAIT,
  START_EXP,
  COLLECT_DATA,
  STOP_EXP,
};

// typedefs

// encapsulates environment data
typedef struct data_st {
  float voltage;
  float current;
  uint16_t local_temperature;
  uint16_t remote_temperature;
} Data;

// encapsulates state information
typedef struct __attribute__((packed)) state_st {
  bool abort;
  bool primed;
  bool pump_powered;
  bool pump_1_on;
  bool pump_2_on;
  bool solenoid_1_open;
  bool solenoid_2_open;
  bool solenoid_3_open;
  bool cell_full;
  bool experimenting;
  bool clean;
  uint8_t rinses;
  enum action current_action;
  enum action last_action;
} LabState;

// end typedefs

#ifdef DEBUG_MODE
#define NOTIFY DEBUG
#define WARNING DEBUG
#define ERROR DEBUG
#endif  // DEBUG_MODE

// global variables

Bonk::StateManager<LabState> sm;
Bonk::LogManager lm;
Bonk::Main226 m226;
Bonk::Experiment226 experiment226;
Bonk::Tmp411 thermometer(TMP411_ADDRESS);
Bonk::Pca9557 containmentPins(BONK_CONTAINMENT9557_ADDRESS);
Bonk::Boost226 boost226;

SdFat sd;   // SD File System

LabState state{ 0 };

FatFile data_file;

Bonk::EventHandler eh;
// end global variables

// initialize pin
void pin_init() {

  // initialize pump pins
  containmentPins.pinMode(PUMP_POWER, OUTPUT);
  containmentPins.pinMode(PUMP_1, OUTPUT);
  containmentPins.pinMode(PUMP_2, OUTPUT);

  // initialize experiment pin
  containmentPins.pinMode(EXPERIMENT, OUTPUT);

  // initialize solenoid pins
  containmentPins.pinMode(SOL_1, OUTPUT);
  containmentPins.pinMode(SOL_2, OUTPUT);
  containmentPins.pinMode(SOL_3, OUTPUT);

  // initialize fluid sensor pins
  containmentPins.pinMode(FLUID_SENSOR_1, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_2, INPUT);

  // valves are active low
  // pumps are active high
  // pump power active low
  // experiment is active low
  containmentPins.digitalWrite(PUMP_POWER, HIGH);
  containmentPins.digitalWrite(PUMP_1, LOW);
  containmentPins.digitalWrite(PUMP_2, LOW);
  containmentPins.digitalWrite(SOL_1, HIGH);
  containmentPins.digitalWrite(SOL_2, HIGH);
  containmentPins.digitalWrite(SOL_3, HIGH);
  containmentPins.digitalWrite(EXPERIMENT, HIGH);

  lm.log(Bonk::LogType::NOTIFY, "pins initialized");
}

// configures and initializes serial, sd,
// pump, solenoid, experiment interfaces
void setup() {
  // initialize Wire for containment pins
  Wire.begin();
  Wire.setClock(400000);

  // initialize Serial connection with New Shepard
  Serial.begin(115200, SERIAL_8N1);

  // initialize SdFat
  sd.begin(CHIP_SELECT);

  lm.begin("log.txt");
  sm.begin("state.txt", state);
  // TDOD: update shunt resistor, current limit values
  m226.begin();
  thermometer.begin();
  containmentPins.begin();
  Bonk::enableBoostConverter(true);
  boost226.begin();

  experiment226.begin(); 

  // configure pins
  lm.log(Bonk::LogType::NOTIFY, "BONK initialized");
  pin_init();

  LabState temp;
  if (sm.get_state(temp)) {
    state = temp;
  }

  // TODO: add extra logic for restoring state/turning on hardware
}

void start_sulf() {
  if (!state.solenoid_1_open) {
    // open the CuSO4 valve
    containmentPins.digitalWrite(SOL_1, LOW);

    // update our state
    state.solenoid_1_open = true;
    sm.set_state(state);

    // log that we opened the solenoid
    lm.log(Bonk::LogType::NOTIFY, "sulfur fill valve open");
  } else if (!state.pump_1_on) {
    // turn on the fill pump
    containmentPins.digitalWrite(PUMP_POWER, LOW);
    containmentPins.digitalWrite(PUMP_1, HIGH);

    // update our state
    state.pump_1_on = true;
    state.pump_powered = true;
    sm.set_state(state);

    // log that we turned on the fill pump
    lm.log(Bonk::LogType::NOTIFY, "fill pump on");
  }
}

void stop_sulf() {
  if (!state.pump_1_on) {
    // turn off the fill pump
    containmentPins.digitalWrite(PUMP_POWER, HIGH);
    containmentPins.digitalWrite(PUMP_1, LOW);

    // update our state
    state.pump_1_on = false;
    state.pump_powered = false;
    sm.set_state(state);

    // log that we turned off the fill pump
    lm.log(Bonk::LogType::NOTIFY, "fill pump off");
  } else if (!state.solenoid_1_open) {
    // close the CuSO4 valve
    containmentPins.digitalWrite(SOL_1, HIGH);

    // update our state
    state.solenoid_1_open = false;
    sm.set_state(state);

    // log that we closed the solenoid
    lm.log(Bonk::LogType::NOTIFY, "sulfur fill valve closed");
  }
}

bool check_full() {
  return containmentPins.digitalRead(FLUID_SENSOR_1) == HIGH;
}

bool check_drain() {
  return containmentPins.digitalRead(FLUID_SENSOR_2) == LOW;
}

void start_water() {
  if (!state.solenoid_2_open) {
    // open the CuSO4 valve
    containmentPins.digitalWrite(SOL_2, LOW);

    // update our state
    state.solenoid_2_open = true;
    sm.set_state(state);

    // log that we opened the solenoid
    lm.log(Bonk::LogType::NOTIFY, "sulfur fill valve open");
  } else if (!state.pump_1_on) {
    // turn on the fill pump
    containmentPins.digitalWrite(PUMP_POWER, LOW);
    containmentPins.digitalWrite(PUMP_1, HIGH);

    // update our state
    state.pump_1_on = true;
    state.pump_powered = true;
    sm.set_state(state);

    // log that we turned on the fill pump
    lm.log(Bonk::LogType::NOTIFY, "fill pump on");
  }
}

void stop_water() {
  if (!state.pump_1_on) {
    // turn off the fill pump
    containmentPins.digitalWrite(PUMP_POWER, HIGH);
    containmentPins.digitalWrite(PUMP_1, LOW);

    // update our state
    state.pump_1_on = false;
    state.pump_powered = false;
    sm.set_state(state);

    // log that we turned off the fill pump
    lm.log(Bonk::LogType::NOTIFY, "fill pump off");
  } else if (!state.solenoid_2_open) {
    // close the water valve
    containmentPins.digitalWrite(SOL_2, HIGH);

    // update our state
    state.solenoid_2_open = false;
    sm.set_state(state);

    // log that we closed the solenoid
    lm.log(Bonk::LogType::NOTIFY, "water fill valve closed");
  }
}

void start_exp() {
  if (!state.experimenting) {
    // turn on the experiment
    containmentPins.digitalWrite(EXPERIMENT, LOW);

    // update our state
    state.experimenting = true;
    sm.set_state(state);

    // log that we started the experiment
    lm.log(Bonk::LogType::NOTIFY, "experiment started");
  }
}

void stop_exp() {
  if (state.experimenting) {
    // turn off the experiment
    containmentPins.digitalWrite(EXPERIMENT, HIGH);

    // update our state
    state.experimenting = false;
    sm.set_state(state);

    // log that we started the experiment
    lm.log(Bonk::LogType::NOTIFY, "experiment stopped");
  }
}

void collect_data() {
  static unsigned long last_log_time = millis();
  if (millis() - last_log_time >= LOG_TIME_CUTOFF) {
    float current = experiment226.readShuntCurrent();
    float voltage = experiment226.readShuntVoltage();
    uint16_t local_temperature = thermometer.readLocalTemperature();
    uint16_t remote_temperature = thermometer.readRemoteTemperature();
    data_file.printField(voltage, ',', 4);
    data_file.printField(current, ',', 4);
    data_file.printField(local_temperature, ',');
    data_file.printField(remote_temperature, '\n');
    data_file.flush();    // TODO: might take too much time, needs characterization
    last_log_time = millis();
  }
}

void start_drain() {
  if (!state.solenoid_1_open) {
    // open the drain valve
    containmentPins.digitalWrite(SOL_3, LOW);

    // update our state
    state.solenoid_3_open = true;
    sm.set_state(state);

    // log that we opened the solenoid
    lm.log(Bonk::LogType::NOTIFY, "drain valve open");
  } else if (!state.pump_2_on) {
    // turn on the fill pump
    containmentPins.digitalWrite(PUMP_POWER, LOW);
    containmentPins.digitalWrite(PUMP_2, HIGH);

    // update our state
    state.pump_2_on = true;
    state.pump_powered = true;
    sm.set_state(state);

    // log that we turned on the drain pump
    lm.log(Bonk::LogType::NOTIFY, "drain pump on");
  }
}

void stop_drain() {
  if (!state.pump_2_on) {
    // turn off the drain pump
    containmentPins.digitalWrite(PUMP_POWER, HIGH);
    containmentPins.digitalWrite(PUMP_2, LOW);

    // update our state
    state.pump_2_on = false;
    state.pump_powered = false;
    sm.set_state(state);

    // log that we turned off the drain pump
    lm.log(Bonk::LogType::NOTIFY, "drain pump off");
  } else if (!state.solenoid_2_open) {
    // close the drain valve
    containmentPins.digitalWrite(SOL_3, HIGH);

    // update our state
    state.solenoid_2_open = false;
    sm.set_state(state);

    // log that we closed the solenoid
    lm.log(Bonk::LogType::NOTIFY, "drain valve closed");
  }
}

void tick_fsm(Bonk::FlightEvent fe) {
  // FSM Transitions
  switch (state.current_action) {
    case action::IDLE:
      if (fe == Bonk::FlightEvent::Liftoff) {
        state.current_action = action::START_SULF;
      } else if (fe == Bonk::FlightEvent::CoastStart) {
        state.current_action = action::START_EXP;
      } else {
        state.current_action = action::IDLE;
      }
      break;

    case action::START_SULF:
      if (state.pump_1_on) {
        state.current_action = action::CHECK_FULL;
      } else {
        state.current_action = action::START_SULF;
      }
      break;

    case action::STOP_SULF:
      if (!state.pump_1_on) {
        state.current_action = action::IDLE;
      } else {
        state.current_action = action::STOP_SULF;
      }
      break;

    case action::CHECK_FULL:
      if (check_full()) {
        if (state.last_action == action::START_SULF) {
          state.current_action = action::STOP_SULF;
        } else if (state.last_action == action::START_WATER) {
          state.current_action = action::STOP_WATER;
        }
      } else {
        state.current_action = action::CHECK_FULL;
      }
      break;
      
    case action::START_EXP:
      if (state.experimenting) {
        state.current_action = action::COLLECT_DATA;
      } else {
        state.current_action = action::START_EXP;
      }
      break;

    case action::COLLECT_DATA:
      if (fe == Bonk::FlightEvent::CoastEnd) {
        state.current_action = action::STOP_EXP;
      } else {
        state.current_action = action::COLLECT_DATA;
      }
      break;

    case action::STOP_EXP:
      if (!state.experimenting) {
        state.current_action = action::START_DRAIN;
      } else {
        state.current_action = action::STOP_EXP;
      }
      break;

    case action::START_DRAIN:
      if (state.pump_2_on) {
        state.current_action = action::DRAIN_WAIT;
      } else {
        state.current_action = action::START_DRAIN;
      }
      break;

    case action::DRAIN_WAIT:
      static unsigned long drain_time;
      if (state.last_action == action::START_DRAIN) {
        drain_time = millis();
      }

      if (millis() - drain_time >= DRAIN_TIME_CUTOFF) {
        state.current_action = action::CHECK_DRAIN;
      } else {
        state.current_action = action::DRAIN_WAIT;
      }
      break;

    case action::CHECK_DRAIN:
      if (check_drain()) {
        state.current_action = action::STOP_DRAIN;
      } else {
        state.current_action = action::CHECK_DRAIN;
      }
      break;

    case action::STOP_DRAIN:
      if (!state.pump_2_on) {
        if (state.rinses == RINSES) {
          state.current_action = action::IDLE;
        } else {
          state.current_action = action::START_WATER;
        }
      } else {
        state.current_action = action::STOP_DRAIN;
      }
      break;

    case action::START_WATER:
      if (state.pump_1_on) {
        state.current_action = action::CHECK_FULL;
      } else {
        state.current_action = action::START_WATER;
      }
      break;

    case action::STOP_WATER:
      if (!state.pump_1_on) {
        state.current_action = action::START_DRAIN;
      } else {
        state.current_action = action::STOP_WATER;
      }
      break;
  }

  // FSM Actions
  switch (state.current_action) {
    case action::START_SULF:
      start_sulf();
      break;

    case action::STOP_SULF:
      stop_sulf();
      break;

    case action::START_EXP:
      start_exp();
      break;

    case action::COLLECT_DATA:
      collect_data();
      break;

    case action::STOP_EXP:
      stop_exp();
      break;

    case action::START_DRAIN:
      start_drain();
      break;

    case action::STOP_DRAIN:
      stop_drain();
      break;

    case action::START_WATER:
      start_water();
      break;

    case action::STOP_WATER:
      stop_water();
      break;
  }
  
  if (state.last_action != state.current_action) {
    sm.set_state(state);
  }
  state.last_action = state.current_action;
}

void loop() {
  // tick BONK
  eh.tick();

  // get the most recent New Shepard flight event
  Bonk::ShipReading last_reading = eh.getLastReading();
  
 
}
