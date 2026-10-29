#include <Servo.h>

#define BREAK_WIRE_PIN  7
#define SERVO_PIN       2

const unsigned long DEPLOY_DELAY_MS = 12500;  // delay after wire break before servo actuates

Servo deployServo;

enum DeployState { ARMED, WIRE_BROKEN, DEPLOYING, DEPLOYED };
DeployState deployState = ARMED;

unsigned long wireBreakTime = 0;

void setup() {
  pinMode(BREAK_WIRE_PIN, INPUT_PULLUP);

  deployServo.attach(SERVO_PIN);
  deployServo.write(90);  // hold position

  Serial.begin(9600);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000);
  Serial.println("Armed. Waiting for break wire...");
}

void loop() {
  unsigned long now = millis();

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
        deployState = DEPLOYING;
        deployServo.write(180);  // release position
        Serial.println("Deploying!");
      }
      break;

    case DEPLOYING:
      deployState = DEPLOYED;
      break;

    case DEPLOYED:
      break;
  }
}
