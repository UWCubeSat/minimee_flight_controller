// Eli Reed, November 18th 2019
// written for the NanoLab mission

// Controller for the electroplating experiment we're running
// for our WASGC-1 Mission (name to-be-changed hopefully).

// Blue Serial Connection Spec:
// Baud Rate: 115200
// Data: 8 bits |
// Parity: None |--> 8N1
// Stop Bits: 1 |
// 
// Packets are ASCII string of 21 comma-separated values
// size of packets <= 250 bytes (buffer is 256 bytes), individual fields will never exceed 20 bytes
// Data stream starts approximately one minute after nano-labs are powered up

#include "blue_origin_fc.h"
#include <SPI.h>
#include <SD.h>
//#include <TimerOne.h>

int lab_state;
String blue_state;
int last_lab_state;
float last_blue_time = 0;

unsigned long last_log_time = 0;

unsigned long pump_start_time = 0;

bool pump_on = false;

bool plating_started = false;

bool pump_fill = true;

bool pump_empty = true;

bool on_pad = true;

const bool DEBUG = true;

void setup() {
  // initialize serial interface
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);

  File lf;
  if (!SD.begin(CHIP_SELECT)) {
    log_event("SD failed to initialize", lf);
  } else {
    lf = SD.open(LOG_FILE, FILE_WRITE);
    log_event("SD initialized", lf);
  }
  log_event("Serial initialized.", lf);
  
  // determine if we need to read last state
  if (SD.exists(STATE_FILE)) {
    log_event("State file found, reloading state.", lf);
    //restore_state();
  } else {
    log_event("No state file found.", lf);
    lab_state = LS_IDLE;
    blue_state = BS_NO_STATE;
  }

  // configure pins
  pinMode(PUMP_POWER, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(EXPERIMENT, OUTPUT);
  digitalWrite(PUMP_POWER, LOW);
}

void loop() {
  switch(lab_state) {
    case LS_IDLE:
      {     
        if (Serial.available() > 0) {
          last_lab_state = LS_IDLE;
          lab_state = LS_SERIAL_READ;
          break;
        }
        unsigned long t = millis();
        if (t - last_log_time >= 1000) {
          last_log_time = t;
          last_lab_state = LS_IDLE;
          lab_state = LS_LOGGING;
          break;
        }
        if (blue_state.equals(BS_COAST_START)) {
          last_lab_state = LS_IDLE;
          lab_state = LS_CELL_PLATING;
          break;
        }
        if (blue_state.equals(BS_NO_STATE)) {
          if () {
            last_lab_state = LS_IDLE;
            lab_state = LS_PUMP_FILL;
            pump_fill = false;
            break; 
          }
        }
        if (blue_state.equals(BS_LANDING) {
          if (pump_empty
        }
        last_lab_state = LS_IDLE;
        lab_state = LS_IDLE;
      }
      break;
    case LS_SERIAL_READ:
      {
        File lf = SD.open(LOG_FILE, FILE_WRITE);
        delay(20);
        for (int i = 0; i < 21; i++) {
          char buf[21];
          int bytes = 0;
          char next = Serial.read();
          while (next > -1 && bytes < 21 && next != ',') {
            buf[bytes] = next;
            bytes++;
            next = Serial.read();
          }
//          for (bytes = 0; bytes <= 21; bytes++) {
//            if (Serial.peek() > -1) {
//              char next = Serial.read();
//              if (next == ',') {
//                break;
//              } else {
//                buf[bytes] = next;
//              }
//            } else {
//              break;
//            }
//          }
          buf[bytes] = '\0';
          if (i == 0) {
            blue_state = buf;
          }
          if (i == 1) {
            last_blue_time = ((String)buf).toFloat();
          }
        }
        lab_state = last_lab_state;
        last_lab_state = LS_SERIAL_READ;
        lf.close();
      }
      break;
      
    case LS_PUMP_FILL:
      {
        // set pump power high
        lab_state = last_lab_state;
        digitalWrite(PUMP_1, HIGH);
        last_lab_state = LS_PUMP_FILL;
      }
      break;
      
    case LS_PUMP_EMPTY:
      {
        lab_state = last_lab_state;
        digitalWrite(PUMP_2, HIGH);
        last_lab_state = LS_PUMP_EMPTY;
      }
      break;
      
    case LS_CELL_PLATING:
      {
        File lf = SD.open(LOG_FILE, FILE_WRITE);
        if (blue_state.equals(BS_COAST_START) || blue_state.equals(BS_APOGEE)) {
          if (!plating_started) {
            // start plating
            digitalWrite(PUMP_POWER, LOW);
            digitalWrite(EXPERIMENT, HIGH);
            plating_started = true;
            lf.close();
          }
          if (Serial.available() > 0) {
            last_lab_state = LS_CELL_PLATING;
            lab_state = LS_SERIAL_READ;
            break;
          }
          unsigned long t = millis();
          if (t - last_log_time >= 1000) {
            last_log_time = t;
            last_lab_state = LS_CELL_PLATING;
            lab_state = LS_LOGGING;
            break;
          }
        } else {
          digitalWrite(EXPERIMENT, LOW);
          File lf = SD.open(LOG_FILE, FILE_WRITE);
          log_event("Plating ended", lf);
          lf.close();          
          lab_state = LS_IDLE;
          last_lab_state = LS_CELL_PLATING;
        }
      }
      break;
      
    case LS_LOGGING:
      {
        File lf = SD.open(LOG_FILE, FILE_WRITE);
        float curr_val = analogRead(CURR_ANALOG_PIN) * (5.0 / 1023.0);
        float volt_val = 5.0 - analogRead(VOLT_ANALOG_PIN) * (5.0 / 1023.0);
        float temp_val = analogRead(TEMP_ANALOG_PIN) * (5.0 / 1023.0) * 0.01;
        write_data_to_csv(volt_val, curr_val, temp_val);
        save_state();
        log_event("Blue is in state: " + blue_state + " after " + String(last_blue_time) + " seconds.", lf);
        lf.close();
        lab_state = last_lab_state;
        last_lab_state = LS_LOGGING;
      }
      break;
    default:
      {
        File lf = SD.open(LOG_FILE, FILE_WRITE);
        log_event("Reached a null state. Last lab state: " + String(last_lab_state), lf);
        lab_state = LS_IDLE;
        last_lab_state = LS_NO_STATE;
        lf.close();
      }
      break;
  }
}

void log_event(String message, File f) {
  if (DEBUG) {
    Serial.println(get_time_stamp() + " " + message);
  } else {
    f.println(get_time_stamp() + " " + message);
  }
}

void write_data_to_csv(float volt, float curr, float temp) {
  File df = SD.open(DATA_FILE, FILE_WRITE);
  if (DEBUG) {
    Serial.println(get_time_stamp() + "," + String(volt) + "," + String(curr) + "," + String(temp));
  } else {
    df.println(get_time_stamp() + "," + String(volt) + "," + String(curr) + "," + String(temp));
  }
  df.close();
}

void save_state() {
  if (SD.exists(STATE_FILE)) {
    SD.remove(STATE_FILE);
  }
  File sf = SD.open(STATE_FILE, FILE_WRITE);
  if (DEBUG) {
    Serial.println(get_time_stamp() + "," + lab_state + "," + blue_state);
  } else {
    sf.println(get_time_stamp() + "," + lab_state + "," + blue_state);
  }
  sf.close();
}

String get_time_stamp() {
  unsigned long t = millis();
  unsigned long seconds = t / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long mils = t % 1000;
  seconds = seconds % 60;
  String s_min = String(minutes);
  String s_sec = String(seconds);
  String s_mils = String(mils);
  return "[" + s_min + ":" + s_sec + ":" + s_mils + "]";
}
