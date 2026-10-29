#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "MS5611.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

const int chipSelect = 10;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);  // I2C

MS5611 baro(0x77);
const float SEA_LEVEL_PRESSURE = 1013.25;

// ── Logging interval ──────────────────────────────────────────────────────────
const unsigned long LOG_INTERVAL_MS = 100;  // log every 100ms (10 Hz)
unsigned long lastLogTime = 0;

// ── Apogee detection ──────────────────────────────────────────────────────────
const float LAUNCH_ALTITUDE_THRESHOLD = 10.0;
const float APOGEE_PRESSURE_DELTA     = 1.5;
const int   APOGEE_CONFIRM_SAMPLES    = 8;
const int   PRESSURE_WINDOW_SIZE      = 6;
const int PYRO_PIN = 9;  // must not share a pin with chipSelect
float apogeeAltitude = 0.0;

enum FlightState { PAD, ASCENDING, APOGEE_DETECTED, DESCENT };
FlightState state = PAD;

float minPressure        = 9999.0;
float padAltitude        = 0.0;
int   apogeeConfirmCount = 0;

float pressureBuf[PRESSURE_WINDOW_SIZE] = {0};
int   pressureBufIdx  = 0;
bool  pressureBufFull = false;

float updatePressureAvg(float newPressure) {
  pressureBuf[pressureBufIdx] = newPressure;
  pressureBufIdx = (pressureBufIdx + 1) % PRESSURE_WINDOW_SIZE;
  if (pressureBufIdx == 0) pressureBufFull = true;
  int   count = pressureBufFull ? PRESSURE_WINDOW_SIZE : pressureBufIdx;
  float sum   = 0;
  for (int i = 0; i < count; i++) sum += pressureBuf[i];
  return sum / count;
}
// ─────────────────────────────────────────────────────────────────────────────


void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}


void setup() {
  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  if (!baro.begin()) {
    Serial.println("MS5611 not found, check wiring/address!");
    while (1);
  }
  Serial.println("ADXL375 Accelerometer Test"); Serial.println("");
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while(1);
  }

  accel.printSensorDetails();
  displayDataRate();
  Serial.println("");

  // ── Warm up pressure buffer and record pad altitude ───────────────────────
  for (int i = 0; i < PRESSURE_WINDOW_SIZE * 3; i++) {
    baro.read();
    updatePressureAvg(baro.getPressure());
    delay(50);
  }
  baro.read();
  padAltitude = 44330.0 * (1.0 - pow(updatePressureAvg(baro.getPressure()) / SEA_LEVEL_PRESSURE, 1.0 / 5.255));
  Serial.print("Pad altitude: "); Serial.print(padAltitude, 1); Serial.println(" m — Ready.");
  // ─────────────────────────────────────────────────────────────────────────

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }
  Serial.println("initialization done.");
}

void loop() {
  //MS5611
  baro.read();
  float pressure    = baro.getPressure();
  float temperature = baro.getTemperature();
  float altitude    = 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 1.0 / 5.255));
  float smoothPressure = updatePressureAvg(pressure);

  // ── Apogee state machine (runs every iteration) ───────────────────────────
  switch (state) {
    case PAD:
      minPressure = smoothPressure;
      if (altitude - padAltitude > LAUNCH_ALTITUDE_THRESHOLD) {
        state = ASCENDING;
      }
      break;
    case ASCENDING:
      if (smoothPressure < minPressure) {
        minPressure = smoothPressure;
        apogeeConfirmCount = 0;
      } else if (smoothPressure > minPressure + APOGEE_PRESSURE_DELTA) {
        if (++apogeeConfirmCount >= APOGEE_CONFIRM_SAMPLES) {
          state = APOGEE_DETECTED;
        }
      }
      break;
    case APOGEE_DETECTED:
      apogeeAltitude = altitude;
      Serial.print("Apogee alt: "); Serial.println(apogeeAltitude, 2);
      digitalWrite(PYRO_PIN, HIGH);  // pyrocharge fire!
      state = DESCENT;
      break;
    case DESCENT:
      break;
  }
  // ─────────────────────────────────────────────────────────────────────────

  //ADXL375
  sensors_event_t event;
  accel.getEvent(&event);

  // ── Log at fixed interval ─────────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;

    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.print("Temp: "); dataFile.print(temperature, 2);
      dataFile.print("\tAlt: "); dataFile.print(altitude, 2);
      dataFile.print(" m\tState: "); dataFile.println(state);

      dataFile.print("X: "); dataFile.print(event.acceleration.x); dataFile.print("  ");
      dataFile.print("Y: "); dataFile.print(event.acceleration.y); dataFile.print("  ");
      dataFile.print("Z: "); dataFile.print(event.acceleration.z); dataFile.print("  "); dataFile.println("m/s^2 ");

      if (state == DESCENT) {
        dataFile.print("\tApogee: "); dataFile.print(apogeeAltitude, 1); dataFile.print(" m");
      }

      dataFile.close();
    } 
    else {
      Serial.println("error opening datalog.txt");
    }
  }
  // ─────────────────────────────────────────────────────────────────────────
}