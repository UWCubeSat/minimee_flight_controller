#include <Wire.h>
#include <BonkFramework.h>

#define MCP4726_I2C_ADDR_1 0b1100000
#define MCP4726_I2C_ADDR_2 0b1100001
#define WRITE_VOL_MEM (0b010 << 5)
#define WRITE_VOL_REG (0b000 << 5)
#define WRITE_ALL_MEM (0b011 << 5)
#define WRITE_VOL_CONFIG (0b100 << 5)
#define CLOCK_SPEED 400000

#define FLUID_SENSOR_1_ON 0  // when first sensor is definitely on
#define FLUID_SENSOR_1_OFF 1  // when it's definitely off
#define FLUID_SENSOR_2_ON 2   // when second sensor is definitely on
#define FLUID_SENSOR_2_OFF 3  // when it's definitely off

bool dead = false;
uint8_t buf[3] = {0, 0, 0};
Bonk::Pca9557 containmentPins(BONK_CONTAINMENT9557_ADDRESS);

void setup() {
  // set up wire
  Serial.begin(115200);

  if (!Serial) {
    dead = true;
    return;
  }
  
  Wire.begin();
  Wire.setClock(CLOCK_SPEED);
  containmentPins.begin();
  containmentPins.pinMode(FLUID_SENSOR_1_ON, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_1_OFF, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_2_ON, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_2_OFF, INPUT);
  
  // us Vdd as Vref (ignore Vref pin)
  buf[0] = WRITE_ALL_MEM;
  Wire.beginTransmission(MCP4726_I2C_ADDR_1);
  Wire.write(buf, sizeof(buf) / sizeof(uint8_t));
  Wire.endTransmission();

  Wire.beginTransmission(MCP4726_I2C_ADDR_2);
  Wire.write(buf, sizeof(buf) / sizeof(uint8_t));
  Wire.endTransmission();
}

bool check_full() {
  bool def_on = containmentPins.digitalRead(FLUID_SENSOR_1_ON) == HIGH;
  bool def_off = containmentPins.digitalRead(FLUID_SENSOR_1_OFF) == HIGH;
  return def_on && def_off;
}

bool check_drain() {
  bool def_on = containmentPins.digitalRead(FLUID_SENSOR_2_ON) == HIGH;
  bool def_off = containmentPins.digitalRead(FLUID_SENSOR_2_OFF) == HIGH;
  return def_on && def_off;
}

void loop() {
  uint16_t setpoint_1 = 0;
  uint16_t setpoint_2 = 0;
  if (!dead && Serial.available() > 0) {
    char read_buf[16];
    size_t n = Serial.readBytesUntil(',', read_buf, 5);
    Serial.println(n);
    char* out;
    setpoint_1 = strtol(read_buf, &out, 10);
    Serial.read();  // discard the space
    n = Serial.readBytesUntil('\n', read_buf, 5);
    Serial.println(n);
    setpoint_2 = strtol(read_buf, &out, 10);

    Serial.println(setpoint_1);
    Serial.println(setpoint_2);

    buf[0] = WRITE_ALL_MEM;
    buf[1] = (uint8_t)((setpoint_1 >> 8) & 0x0f);
    buf[2] = (uint8_t)(setpoint_1);
    
    Wire.beginTransmission(MCP4726_I2C_ADDR_1);
    Wire.write(buf, sizeof(buf) / sizeof(uint8_t));
    Wire.endTransmission();

    buf[1] = (uint8_t)((setpoint_2 >> 8) & 0x0f);
    buf[2] = (uint8_t)(setpoint_2);

    Wire.beginTransmission(MCP4726_I2C_ADDR_2);
    Wire.write(buf, sizeof(buf) / sizeof(uint8_t));
    Wire.endTransmission();

    bool def_on = containmentPins.digitalRead(FLUID_SENSOR_1_ON) == HIGH;
    bool def_off = containmentPins.digitalRead(FLUID_SENSOR_1_OFF) == HIGH;
    Serial.print("Fluid sensor 1: ");
    Serial.print(def_on);
    Serial.print(", ");
    Serial.print(def_off);
    Serial.println();

    def_on = containmentPins.digitalRead(FLUID_SENSOR_2_ON) == HIGH;
    def_off = containmentPins.digitalRead(FLUID_SENSOR_2_OFF) == HIGH;
    Serial.print("Fluid sensor 2: ");
    Serial.print(def_on);
    Serial.print(", ");
    Serial.print(def_off);
    Serial.println();
  }
}
