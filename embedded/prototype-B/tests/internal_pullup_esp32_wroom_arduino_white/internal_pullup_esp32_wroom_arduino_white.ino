#include <Wire.h>

#define SDA_PIN     25
#define SCL_PIN     26
#define RESET_PIN   18

#define TLC59108_0_ADDR 0x40  // Default I2C address
#define TLC59108_2_ADDR 0x42  // Default I2C address

// TLC59108 register addresses
#define MODE1       0x00
#define MODE2       0x01
#define PWM0        0x02  // PWM0 to PWM7: 0x02 - 0x09
#define LEDOUT0     0x0C
#define LEDOUT1     0x0D

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure reset pin
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);   // Hold RESET low
  delay(10);                      // Wait for device reset
  digitalWrite(RESET_PIN, HIGH);  // Release RESET
  delay(10);

  // Start I2C on custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Initializing TLC59108...");

  // Configure MODE1 and MODE2
  writeRegister(TLC59108_0_ADDR, MODE1, 0x00); // Normal mode
  writeRegister(TLC59108_0_ADDR, MODE2, 0x04); // Outputs change on STOP, totem-pole output

  // Configure MODE1 and MODE2
  writeRegister(TLC59108_2_ADDR, MODE1, 0x00); // Normal mode
  writeRegister(TLC59108_2_ADDR, MODE2, 0x04); // Outputs change on STOP, totem-pole output
  
  // Set all PWM channels
  for (uint8_t i = 0; i < 8; i++) {
    writeRegister(TLC59108_0_ADDR, PWM0 + i, 0xFF);
    writeRegister(TLC59108_2_ADDR, PWM0 + i, 0xFF);
  }

  // Set LEDOUTx registers to PWM control (0xAA = PWM mode)
  writeRegister(TLC59108_0_ADDR, LEDOUT0, 0xAA);  // LED0–3 use PWM
  writeRegister(TLC59108_0_ADDR, LEDOUT1, 0xAA);  // LED4–7 use PWM
  
  // Set LEDOUTx registers to PWM control (0xAA = PWM mode)
  writeRegister(TLC59108_2_ADDR, LEDOUT0, 0xAA);  // LED0–3 use PWM
  writeRegister(TLC59108_2_ADDR, LEDOUT1, 0xAA);  // LED4–7 use PWM

  Serial.println("TLC59108 setup complete. LEDs should be on at full brightness.");
}

void loop() {
  // Do nothing
}
