#define CHIP_SELECT 10
#define TEMP_ANALOG_PIN A0
#define CURR_ANALOG_PIN A1
#define VOLT_ANALOG_PIN A2

#define PUMP_POWER 4
#define PUMP_1 5
#define PUMP_2 6

#define EXPERIMENT 9

#define MAX_FRAME_SIZE 250
#define MAX_FIELD_SIZE 20

#define LS_NO_STATE 0
#define LS_IDLE 1
#define LS_SERIAL_READ 2
#define LS_PUMP_FILL 3
#define LS_PUMP_EMPTY 4
#define LS_CELL_PLATING 5
#define LS_LOGGING 6

#define LOG_FILE "log.txt"
#define STATE_FILE "state.txt"
#define DATA_FILE "data.csv"

#define BS_NO_STATE "@"
#define BS_ABORT_ENABLED "A"
#define BS_ABORT_COMMANDED "B"
#define BS_LIFTOFF "C"
#define BS_MECO "D"
#define BS_SEP_COMMANDED "E"
#define BS_COAST_START "F"
#define BS_APOGEE "G"
#define BS_COAST_END "H"
#define BS_DROGUE_DEPLOY "I"
#define BS_MAIN_CHUTE_DEPLOY "J"
#define BS_LANDING "K"
#define BS_SAFING "L"
#define BS_MISSION_END "M"
