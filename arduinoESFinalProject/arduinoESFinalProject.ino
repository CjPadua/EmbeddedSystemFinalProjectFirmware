// ===== Wind Direction Sensor (RS485 Modbus-RTU) =====

#define RE_DE_PIN 2          // MAX485 RE+DE
HardwareSerial &rs485 = Serial1;  // TX1=18, RX1=19

const uint8_t  SLAVE_ID   = 0x01;
const uint8_t  FUNC_READ  = 0x03;
const uint16_t START_REG  = 0x0000;  // 0000H: angle*10, 0001H: direction code
const uint16_t NUM_REGS   = 2;

const unsigned long TRANSMIT_INTERVAL_MS = 500;
const unsigned long RESPONSE_TIMEOUT_MS = 500;

unsigned long lastReq = 0;

// ===== Wind Speed Sensor =====
#define windSpeedPin A0

// ===== BMP280 Sensor =====
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// BMP280 SPI pins
#define BMP_SCK   52
#define BMP_MISO  50
#define BMP_MOSI  51
#define BMP_CS    53

// Initialize BMP280 using SPI
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

float pressure;

// ===== DHT11 Sensor =====
#include <DHT.h>

#define DHTPIN 3          // Pin where the DHT11 is connected
#define DHTTYPE DHT11     // DHT 11
DHT dht(DHTPIN, DHTTYPE);

float humidity, temperature;

// ===== RAIN Sensor =====
#define rainSensorPin A1

// ===== LDR Sensor =====
#define ldrPin A2

// windDirection|windSpeed|pressure|temperature|humidity|rainSensor|ldr
String readings;
bool readingFailed = false;

// ---------- RS485 direction control ----------
void rs485TransmitMode() { digitalWrite(RE_DE_PIN, HIGH); }
void rs485ReceiveMode()  { digitalWrite(RE_DE_PIN, LOW);  }

// ---------- Modbus CRC16 ----------
uint16_t modbusCRC(const uint8_t *buffer, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;   // low byte first on the wire
}

// ---------- 16-direction name ----------
const char *directionName16(uint16_t code) {
  switch (code & 0x0F) {   // 0–15
    case 0:  return "N";
    case 1:  return "NNE";
    case 2:  return "NE";
    case 3:  return "ENE";
    case 4:  return "E";
    case 5:  return "ESE";
    case 6:  return "SE";
    case 7:  return "SSE";
    case 8:  return "S";
    case 9:  return "SSW";
    case 10: return "SW";
    case 11: return "WSW";
    case 12: return "W";
    case 13: return "WNW";
    case 14: return "NW";
    case 15: return "NNW";
    default: return "Unknown";
  }
}

// ---------- Send Modbus request ----------
void sendRequest() {
  uint8_t frame[8];
  frame[0] = SLAVE_ID;
  frame[1] = FUNC_READ;
  frame[2] = highByte(START_REG);
  frame[3] = lowByte(START_REG);
  frame[4] = highByte(NUM_REGS);
  frame[5] = lowByte(NUM_REGS);

  uint16_t crc = modbusCRC(frame, 6);
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);

  // 1) Make sure we start this transaction with a clean buffer
  while (rs485.available()) rs485.read();

  // 2) Transmit request
  rs485TransmitMode();              // TX
  rs485.write(frame, 8);
  rs485.flush();                    // wait until fully sent

  rs485ReceiveMode();               // immediately switch to RX

  // 3) Give the sensor enough time to answer fully
  delay(40);                        // at 4800 bps, 9–10 bytes ≈ 20 ms; 40 ms is safe
}

// ---------- Read & parse response ----------
bool readResponse(uint16_t *regs) {
  const uint8_t expectedLen = 9;
  uint8_t buf[expectedLen];
  uint8_t idx = 0;

  unsigned long start = millis();
  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    while (rs485.available()) {
      if (idx < expectedLen)
        buf[idx++] = rs485.read();
      else
        rs485.read(); // discard overflow
    }
    if (idx == expectedLen) break;
  }

  if (idx != expectedLen) {
    Serial.print("Frame incomplete, got ");
    Serial.println(idx);
    return false;
  }

  // -------- Validate frame ----------
  if (buf[0] != SLAVE_ID) return false;
  if (buf[1] != FUNC_READ) return false;
  if (buf[2] != 4) return false;  // byte count must be 4

  // CRC check
  uint16_t crcRx = (buf[8] << 8) | buf[7];
  uint16_t crcCalc = modbusCRC(buf, 7);
  if (crcRx != crcCalc) {
    Serial.println("CRC mismatch");
    return false;
  }

  // Parse registers
  regs[0] = (buf[3] << 8) | buf[4]; // angle*10
  regs[1] = (buf[5] << 8) | buf[6]; // direction code

  return true;
}

// ---------- Arduino setup / loop ----------
void setup() {
  Serial.begin(115200);                // Serial Monitor
  rs485.begin(4800, SERIAL_8N1);       // sensor default: 4800 8N1
  Serial2.begin(115200);
  pinMode(RE_DE_PIN, OUTPUT);
  rs485ReceiveMode();
  
  // BMP280 init
  if (!bmp.begin()) {
    Serial.println("BMP280 Not Found!");
  }

  // DHT11
  dht.begin();

  // LDR
  pinMode(ldrPin, INPUT);
}

void loop() {

  if (millis() - lastReq < TRANSMIT_INTERVAL_MS) return;

  lastReq = millis();

  // wind direction ----------------------------
  
  sendRequest();
  uint16_t regs[NUM_REGS];

  if (readResponse(regs)) {
    uint16_t dirCode    = regs[1];
    Serial.print(directionName16(dirCode));

    readings += directionName16(dirCode) + "|";

  } else {
    Serial.println("Frame error");
    readingFailed = true;
  }

  // wind speed -----------------------------
  int rawWindSpeed = analogRead(windSpeedPin);
  float voltage = rawWindSpeed * (5.0/1023.0);

  // 0–5 V -> 0–30 m/s  =>  6 m/s per volt 
  float windSpeed = voltage * 6.0;
  Serial.print(windSpeed);

  if(isnan(windSpeed)) {
    readingFailed = true;
  }
  else {
    readings += String(windSpeed) + "|";
  }


  // Pressure (BMP280)
  
  pressure = bmp.readPressure();

  if(isnan(pressure) || pressure < 1) {
    readingFailed = true;
  }
  else {
    readings += String(pressure) + "|";
  }

  // Temperature and Humidity 
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  if(isnan(temperature) || isnan(humidity) || temperature < 1 || humidity < 1) {
    readingFailed = true;
  }
  else {
    readings += String(temperature) + "|";
    readings += String(humidity) + "|";
  }

  // Rain Sensor
  int rainSensorValue = analogRead(rainSensorPin);

  if(isnan(rainSensorValue) || rainSensorValue < 1) {
    readingFailed = true;
  }
  else {
    readings += String(rainSensorValue) + "|";
  }  
  

  // light sensor
  
  int ldrReading = analogRead(ldrPin);

  if(isnan(ldrReading) || ldrReading < 1) {
    readingFailed = true;
  }
  else {
    readings += String(ldrReading);
  } 
  
  
  if(readingFailed) {
    Serial.println("Reading Failed.");
    Serial2.println("Reading Failed.");
  }
  else {
    Serial.println(readings);
    Serial2.println(readings);
  }
  

  readings = "";
  readingFailed = false;
}
