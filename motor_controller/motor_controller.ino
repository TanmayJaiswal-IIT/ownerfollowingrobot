// DesiPachinos owner-following robot: Arduino Uno motor controller
// Receives "M,<left>,<right>\n" (-255..255) over USB serial at 115200 baud.
// Sends "D,<cm>\n" (ultrasonic distance, 999 = no echo) to the Pi after every ping (~16 Hz).
// Safety on the Arduino itself:
//   - stops if no command arrives for 500 ms (Python crashed / cable pulled)
//   - blocks forward motion if the ultrasonic sensor sees an obstacle under 20 cm
//
// Wiring (L298N): remove the ENA/ENB jumpers.
//   Left motors (2 in parallel)  -> OUT1/OUT2,  Right motors (2 in parallel) -> OUT3/OUT4
//   ENA -> D5, IN1 -> D6, IN2 -> D7, IN3 -> D8, IN4 -> D9, ENB -> D10
//   HC-SR04: TRIG -> D11, ECHO -> D12, VCC -> 5V, GND -> GND
//   Battery + -> L298N 12V, battery - -> L298N GND AND Arduino GND (common ground)

const int ENA = 5, IN1 = 6, IN2 = 7;
const int ENB = 10, IN3 = 8, IN4 = 9;
const int TRIG = 11, ECHO = 12;

const unsigned long WATCHDOG_MS = 500;
const int STOP_CM = 20;

int cmdL = 0, cmdR = 0;
unsigned long lastCmd = 0, lastPing = 0;
long distCm = 999;
String buf;

void setSide(int en, int a, int b, int v) {
  v = constrain(v, -255, 255);
  digitalWrite(a, v > 0 ? HIGH : LOW);
  digitalWrite(b, v < 0 ? HIGH : LOW);
  analogWrite(en, abs(v));
}

long readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);   // ~4 m timeout
  return us == 0 ? 999 : us / 58;
}

void parseLine(const String &line) {
  if (!line.startsWith("M,")) return;
  int comma = line.indexOf(',', 2);
  if (comma < 0) return;
  cmdL = line.substring(2, comma).toInt();
  cmdR = line.substring(comma + 1).toInt();
  lastCmd = millis();
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { parseLine(buf); buf = ""; }
    else if (buf.length() < 32) buf += c;
  }

  if (millis() - lastPing > 60) {
    distCm = readDistanceCm();
    lastPing = millis();
    // only if there is room: a full TX buffer must never stall motor control or the watchdog
    if (Serial.availableForWrite() >= 8) { Serial.print("D,"); Serial.println(distCm); }
  }

  int l = cmdL, r = cmdR;
  if (millis() - lastCmd > WATCHDOG_MS) { l = 0; r = 0; }        // watchdog
  if (distCm < STOP_CM && (l + r) > 0) { l = 0; r = 0; }         // block forward only

  setSide(ENA, IN1, IN2, l);
  setSide(ENB, IN3, IN4, r);
}
