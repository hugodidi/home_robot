#include <LedControl.h>

// --- Pines ---
#define DIN 11
#define CLK 13
#define CS  10

#define BUZZER_PIN 6
#define POT_PIN A0

LedControl lc = LedControl(DIN, CLK, CS, 1);

// --- Estado ---
float distance_min = 10.0;
unsigned long lastBeepTime = 0;
bool beepState = false;

void setup() {
  Serial.begin(115200);

  // Matriz
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // --- Leer Serial ---
  while (Serial.available()) {
    String msg = Serial.readStringUntil('\n');

    // --- DISTANCIA ---
    if (msg.startsWith("D:")) {
      distance_min = msg.substring(2).toFloat();
    }

    // --- MATRIZ ---
    if (msg.startsWith("M:")) {
      int row = 0;
      int idx = 2;

      while (row < 8 && idx < msg.length()) {
        int comma = msg.indexOf(',', idx);
        if (comma == -1) comma = msg.length();

        byte value = (byte) strtol(msg.substring(idx, comma).c_str(), NULL, 16);
        lc.setRow(0, row, value);

        idx = comma + 1;
        row++;
      }
    }
  }

  // --- BUZZER con umbrales ---
  int pot = analogRead(POT_PIN);
  int volume = map(pot, 0, 1023, 0, 255);
  
  unsigned long now = millis();
  int beepInterval = 0;  // 0 = continuo, >0 = intermitente
  bool shouldBeep = false;

  if (distance_min < 1.0) {
    // < 1m: pitido rápido (400ms on/off)
    beepInterval = 400;
  } else if (distance_min < 1.5) {
    // 1.0 - 1.5m: pitido lento (900ms on/off)
    beepInterval = 900;
  } else {
    // >= 1.5m: apagado
    shouldBeep = false;
    beepInterval = 0;
  }

  // Manejar intermitencia
  if (beepInterval > 0) {
    if (now - lastBeepTime >= beepInterval) {
      beepState = !beepState;
      lastBeepTime = now;
    }
    shouldBeep = beepState;
  }

  // Aplicar al buzzer
  if (shouldBeep) {
    analogWrite(BUZZER_PIN, volume);
  } else {
    analogWrite(BUZZER_PIN, 0);
  }
}