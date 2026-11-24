// Rocky Walker: single-servo bench tester for an ESP32 (no PCA9685 needed).
//
// Wiring (one servo at a time):
//   Battery (2S LiPo) -> XT60 -> fuse holder (5 A fuse for the bench) -> UBEC input
//   UBEC output (jumper on 6 V, CHECK WITH A MULTIMETER FIRST) -> servo red (+) and brown (-)
//   ESP32 GPIO 18 -> servo orange/yellow (signal)
//   ESP32 GND -> UBEC output (-)      <- common ground is required
//   ESP32 powered from your laptop's USB. Never power the servo from the ESP32's pins.
//
// Serial monitor at 115200 baud, line ending "Newline". Commands:
//   c        centre (1500 us): do this BEFORE fitting a horn
//   s        slow sweep 1000 -> 2000 -> 1000 us (about 90 deg each way)
//   w        wide sweep 700 -> 2300 us, slowly. STOP (send 'o') if the servo strains or buzzes hard
//   + / -    nudge by 10 us
//   1350     any number from 500 to 2500 sets that pulse width in microseconds
//   o        off: no pulses, the servo goes limp
//
// Works with ESP32 Arduino core 2.x and 3.x.

const int SERVO_PIN = 18;
const int FREQ_HZ = 50;   // 20 ms servo frame
const int RES_BITS = 16;  // duty resolution
#if ESP_ARDUINO_VERSION_MAJOR < 3
const int LEDC_CH = 0;
#endif

int pulseUs = 1500;

uint32_t usToDuty(int us) {
  return (uint32_t)((uint64_t)us * ((1UL << RES_BITS) - 1) / (1000000UL / FREQ_HZ));
}

void writeDuty(uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(LEDC_CH, duty);
#endif
}

void writeUs(int us) {
  pulseUs = constrain(us, 500, 2500);
  writeDuty(usToDuty(pulseUs));
}

void sweep(int from, int to, int stepUs, int dwellMs) {
  int dir = (to > from) ? 1 : -1;
  for (int us = from; dir * us <= dir * to; us += dir * stepUs) {
    writeUs(us);
    delay(dwellMs);
    if (Serial.available()) return;  // any key aborts
  }
}

void help() {
  Serial.println("\nRocky servo tester: c=centre s=sweep w=wide +/-=10us <500..2500>=set o=off");
}

void setup() {
  Serial.begin(115200);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, FREQ_HZ, RES_BITS);
#else
  ledcSetup(LEDC_CH, FREQ_HZ, RES_BITS);
  ledcAttachPin(SERVO_PIN, LEDC_CH);
#endif
  writeUs(1500);
  help();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd == "c") writeUs(1500);
  else if (cmd == "s") { sweep(pulseUs, 1000, 10, 20); sweep(1000, 2000, 10, 20); sweep(2000, 1500, 10, 20); }
  else if (cmd == "w") { sweep(pulseUs, 700, 10, 30); sweep(700, 2300, 10, 30); sweep(2300, 1500, 10, 30); }
  else if (cmd == "+") writeUs(pulseUs + 10);
  else if (cmd == "-") writeUs(pulseUs - 10);
  else if (cmd == "o") { writeDuty(0); Serial.println("off (limp)"); return; }
  else if (cmd.length() && isDigit(cmd[0])) writeUs(cmd.toInt());
  else { help(); return; }
  Serial.printf("pulse %d us\n", pulseUs);
}
