#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define BREAK_WIRE_PIN  7
#define SERVO_PIN       2
#define SD_CS_PIN       4

const unsigned long LOG_INTERVAL_MS  = 500;
const unsigned long DEPLOY_DELAY_MS  = 2500;  // delay after wire break before servo spins
const unsigned long SERVO_RUN_MS     = 2000;  // how long the servo runs

Adafruit_BME280 bme;
Servo deployServo;
File myFile;
bool sdReady = false;

enum DeployState { ARMED, WIRE_BROKEN, DEPLOYING, DEPLOYED };
DeployState deployState = ARMED;

unsigned long lastLogTime     = 0;
unsigned long wireBreakTime   = 0;
unsigned long deployStartTime = 0;

void setup() {
  Serial.begin(9600);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000);

  pinMode(BREAK_WIRE_PIN, INPUT_PULLUP);

  deployServo.attach(SERVO_PIN);
  deployServo.write(90);  // stop (neutral for continuous rotation servo)

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed! Continuing without logging.");
    sdReady = false;
  } else {
    Serial.println("initialization done.");
    sdReady = true;
  }

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("BME280 connected!");
  }

  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("--- NEW RUN ---");
    myFile.close();
  }
}

void loop() {
  unsigned long now = millis();

  // ── Deployment state machine ───────────────────────────────────────────────
  switch (deployState) {
    case ARMED:
      if (digitalRead(BREAK_WIRE_PIN) == HIGH) {
        deployState   = WIRE_BROKEN;
        wireBreakTime = now;
        Serial.println("Break wire triggered!");
      }
      break;
    case WIRE_BROKEN:
      if (now - wireBreakTime >= DEPLOY_DELAY_MS) {
        deployState       = DEPLOYING;
        deployStartTime   = now;
        deployServo.write(180);  // release position
        Serial.println("Deploying...");
      }
      break;
    case DEPLOYING:
      if (now - deployStartTime >= SERVO_RUN_MS) {
        deployState = DEPLOYED;
        Serial.println("Deployed.");
      }
      break;
    case DEPLOYED:
      break;
  }
  // ──────────────────────────────────────────────────────────────────────────

  // ── Log at fixed interval ─────────────────────────────────────────────────
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;

    float temp     = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float alt      = bme.readAltitude(SEALEVELPRESSURE_HPA);
    float humidity = bme.readHumidity();

    if (sdReady) myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(now);      myFile.print("ms ");
      myFile.print(temp);     myFile.print("*C ");
      myFile.print(pressure); myFile.print("hpa ");
      myFile.print(alt);      myFile.print("m ");
      myFile.print(humidity); myFile.print("% state:");
      myFile.println(deployState);
      myFile.close();
    } else {
      Serial.println("error opening test.txt");
    }
  }
  // ──────────────────────────────────────────────────────────────────────────
}
