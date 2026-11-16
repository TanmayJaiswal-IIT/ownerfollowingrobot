// Plain L298N bench test. No serial protocol, no watchdog, no ultrasonic.
// Wheels OFF the ground. Battery ON and e-stop ON, or the wheels cannot turn.
// USB only powers the Arduino.
//
// Wiring:
//   ENA -> D5, IN1 -> D6, IN2 -> D7, IN3 -> D8, IN4 -> D9, ENB -> D10
//   Remove the ENA and ENB jumper caps. Leave the L298N 5V jumper ON.
//   Battery - to L298N GND and to Arduino GND.
//
// Upload this file. The wheels run the sequence by themselves.
// Optional: Serial Monitor at 115200. Keys: f b l r s

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;

const int SPEED = 200;  // 0..255, high so the motors are sure to start

void setSide(int en, int inA, int inB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (speed < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
  analogWrite(en, abs(speed));
}

void drive(int left, int right) {
  setSide(ENA, IN1, IN2, left);
  setSide(ENB, IN3, IN4, right);
}

void runFor(const char *name, int left, int right, int ms) {
  Serial.println(name);
  drive(left, right);
  delay(ms);
  drive(0, 0);
  delay(400);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  drive(0, 0);

  Serial.begin(115200);
  Serial.println("Bench test starting. Wheels must be off the ground.");
  delay(1500);

  runFor("1 LEFT forward", SPEED, 0, 2000);
  runFor("2 RIGHT forward", 0, SPEED, 2000);
  runFor("3 BOTH forward", SPEED, SPEED, 2000);
  runFor("4 BOTH backward", -SPEED, -SPEED, 2000);
  runFor("5 SPIN right", SPEED, -SPEED, 2000);
  runFor("6 SPIN left", -SPEED, SPEED, 2000);

  Serial.println("Done. Keys: f forward, b back, l left, r right, s stop");
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  char c = Serial.read();
  if (c == 'f') drive(SPEED, SPEED);
  else if (c == 'b') drive(-SPEED, -SPEED);
  else if (c == 'l') drive(SPEED, 0);
  else if (c == 'r') drive(0, SPEED);
  else if (c == 's') drive(0, 0);
}
